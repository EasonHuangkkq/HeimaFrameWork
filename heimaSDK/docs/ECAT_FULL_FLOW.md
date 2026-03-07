# ECAT 整体流程总览（结合当前 config.xml）

> 目标：从“程序启动”到“一帧 PDO 指令发出去再收回来”，按**时间顺序 + 文件 + 函数**梳理一遍，并说明三重缓冲和 Wrapper 在各个阶段的作用。

---

## 1. 启动 & 读取配置（config.xml）

### 1.1 入口：`single_motor_demo.cpp::main`

```cpp
DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
sdk.init(configPath.c_str());          // 读取 config.xml 并完成总线初始化

// 然后进入控制循环：
while (true) {
    sdk.setMotorTarget(targets);       // 写指令 → loong_driver_sdk.cpp
    sdk.getMotorActual(actuals);       // 读反馈 → loong_driver_sdk.cpp
    ...
}
```

### 1.2 DriverSDK 初始化：`loong_driver_sdk.cpp`

```cpp
void DriverSDK::init(char const* xmlFile){
    if (imp.init(xmlFile) < 0){
        printf("imp init failed\n");
        exit(-1);
    }
}
```

进入 `DriverSDK::impClass::init`：

- `configXML = new ConfigXML(xmlFile);`
- 读取电机拓扑：
  ```cpp
  auto motorAlias = configXML->motorAlias();   // 6 个 limb 各自的 alias 列表
  // 根据 limb 长度计算 dofLeg/dofArm/.../dofAll
  dofAll = ...;  // 当前 config 下 = 2（两个 Motor，都在腰部 limb=4）

  if (dofAll > 0){
      drivers = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>[dofAll];
  }
  ```

- 读取 EtherCAT 拓扑（来自 `<ECAT>` 部分）：
  ```cpp
  ecatAlias2type     = configXML->alias2type("ECAT");
  ecatAlias2domain   = configXML->alias2attribute("ECAT", "domain");
  ecatDomainDivision = configXML->domainDivision("ECAT");
  ```

  结合你的 `config.xml`：

  ```xml
  <Slaves>
      <Slave master="0" domain="0" alias="1" type="MT_Device">1</Slave>
      <Slave master="0" domain="1" alias="2" type="MT_Device">1</Slave>
  </Slaves>

  <Domains>
      <Domain master="0" order="0" division="1"/>
      <Domain master="0" order="1" division="1"/>
  </Domains>
  ```

  得到：

  - `ecatAlias2type[0]   = { 1 → "MT_Device", 2 → "MT_Device" }`
  - `ecatAlias2domain[0] = { 1 → 0,           2 → 1           }`
  - `ecatDomainDivision[0] = [1, 1]`（两个域，分频都为 1）

- 创建 EtherCAT 主站对象：
  ```cpp
  for (size_t m = 0; m < ecatAlias2type.size(); ++m){
      ecats.emplace_back(m);   // 调用 ECAT::ECAT(m)
      printf("ecats[%zu] created\n", m);
  }
  ```

---

## 2. ECAT 初始化：绑定从站 → 域 → 三重缓冲 → WrapperPair

### 2.1 ECAT 构造：`ecat.cpp::ECAT::ECAT(int order)`

```cpp
ECAT::ECAT(int const order){
    ...
    alias2type       = ecatAlias2type[order];      // {1:"MT_Device", 2:"MT_Device"}
    alias2domain     = ecatAlias2domain[order];    // {1:0, 2:1}
    domainDivision   = ecatDomainDivision[order];  // [1, 1]
    while (init() < 0){
        clean();
    }
}
```

### 2.2 ECAT::init：请求 master + 准备域数组

```cpp
int ECAT::init(){
    master = ecrt_request_master(order);
    std::stringstream deviceName;
    deviceName << "/dev/EtherCAT" << order;
    fd = open(deviceName.str().c_str(), O_RDWR);

    domains      = new ec_domain_t*[domainDivision.size()];
    domainPtrs   = new unsigned char*[domainDivision.size()];
    domainSizes  = new int[domainDivision.size()];
    rxPDOSwaps   = new SwapList*[domainDivision.size()];
    txPDOSwaps   = new SwapList*[domainDivision.size()];
    ...
}
```

### 2.3 ECAT::check：把“配置里的 alias”与“总线上的从站 index”对齐

```cpp
int ECAT::check(){
    ec_master_info_t masterInfo;
    ecrt_master(master, &masterInfo);
    ...
    alias2slave.clear();

    // 按 alias 排好配置顺序：如 configAliases = [1,2]
    std::vector<int> configAliases;
    for(auto const& p : alias2type){
        configAliases.push_back(p.first);
    }
    std::sort(configAliases.begin(), configAliases.end());

    for (int i = 0; i < masterInfo.slave_count; ++i){
        ecrt_master_get_slave(master, i, &slaveInfo);
        ...
        // 从设备 SDO 读 alias（你的驱动返回 0）
        int alias = readAlias(i, category, ...);

        // 我们启用了按 index 匹配 → 将 alias 0 覆盖成配置里的 1 / 2
        if (ecatMatchByIndex && i < (int)configAliases.size()){
            int cfgAlias = configAliases[i];
            printf(", override alias %d->%d (by index)", alias, cfgAlias);
            alias = cfgAlias;
        } else if (alias == 0 && alias2type.size() == 1){
            alias = alias2type.begin()->first;
        }

        printf(", category %s, alias %d\n", category.c_str(), alias);

        auto it = alias2type.find(alias);
        if (it == alias2type.end()){
            printf("\tdevice with the alias not found/enabled in xml\n");
            continue;
        }
        alias2slave.insert({alias, i});  // alias → 从站 index
    }
    ...
}
```

在你的情况下：

- 从站 0/1 都报 alias=0，通过 `ecatMatchByIndex` 强行覆盖为 alias=1/2；
- 最终得到 `alias2slave = { 1 → 0, 2 → 1 }`。

### 2.4 ECAT::config：注册 PDO、创建三重缓冲、绑定 WrapperPair

1. 为每个域创建 EtherCAT domain：

```cpp
for (int d = 0; d < domainDivision.size(); ++d){
    domains[d] = ecrt_master_create_domain(master);
}
```

2. 针对每个 alias（1、2）注册 PDO，并初始化对应 `drivers[alias-1]`：

```cpp
for (auto const& [alias, slave] : alias2slave){
    int domain = alias2domain.find(alias)->second;       // 1→0, 2→1
    std::string type = alias2type.find(alias)->second;   // "MT_Device"

    printf("master %d, domain %d, slave %d, alias %d, category %s, type %s\n", ...);

    // 根据 <RxPDOs>/<TxPDOs> 生成 pdoEntries/pdoInfos，省略
    // 注册 PDO 到对应 domain，得到 offset：
    int rxPDOOffset = ecrt_slave_config_reg_pdo_entry(...);   // 0
    int txPDOOffset = ecrt_slave_config_reg_pdo_entry(...);   // 16
    printf("\trxPDOOffset %d, txPDOOffset %d\n", rxPDOOffset, txPDOOffset);

    // 创建 SDO/REG handler
    ec_sdo_request_t* sdoHandler = ecrt_slave_config_create_sdo_request(...);
    ec_reg_request_t* regHandler = ecrt_slave_config_create_reg_request(...);

    // 初始化对应电机的 WrapperPair
    drivers[alias - 1].init(
        "ECAT", 0, order, domain, slave, alias, type,
        rxPDOOffset, txPDOOffset,
        sdoHandler, regHandler);
}
```

3. 激活 master 后，为每个域创建三重缓冲并绑定到各个 `WrapperPair`：

```cpp
ecrt_master_activate(master);

for (int d = 0; d < domainDivision.size(); ++d){
    domainPtrs[d]  = ecrt_domain_data(domains[d]);
    domainSizes[d] = ecrt_domain_size(domains[d]); // 你现在两个域都是 32

    printf("master %d, domain %d, domainPtr %ld, domainSize %d\n", ...);

    // 创建 Rx/Tx 三重缓冲
    rxPDOSwaps[d] = new SwapList(domainSizes[d]);
    txPDOSwaps[d] = new SwapList(domainSizes[d]);

    // 遍历所有电机，把属于当前 domain 的绑定到这个 SwapList 上
    for (int j = 0; j < dofAll; ++j){
        switch (drivers[j].config("ECAT", order, d, rxPDOSwaps[d], txPDOSwaps[d])){
        case 0:  // 成功绑定
            break;
        case 1:  // 不是这个 domain
        case 2:  // 还没 init
            break;
        case -1:
            printf("\tdrivers[%d] config failed\n", j);
            return -1;
        }
    }
}
```

到此：

- 每个域 d 有自己的三重缓冲：`rxPDOSwaps[d]` / `txPDOSwaps[d]`（大小=domainSize=32）。
- 每个电机 `drivers[i]`：
  - 知道自己属于哪个 master/domain/slave/alias；
  - `rx/tx` 两个 `DataWrapper` 已经设置了 offset（0/16）和对应的 `SwapList*`；
  - `parameters` 里有从 xml 读出的极性/齿比/额定电流等。

### 2.5 启动 EtherCAT 线程：`ECAT::run`

```cpp
int ECAT::run(){
    // 创建 rxtx 线程并设置 CPU 亲和力
    pthread_create(&pth, nullptr, &rxtx, this);
    pthread_detach(pth);

    // 把所有从站切到 OP
    for (auto const& [alias, slave] : alias2slave){
        while (requestState(slave, "OP") < 0);
    }
}
```

此时：主线程负责调用 `setMotorTarget` / `getMotorActual`，`rxtx` 线程负责和 EtherCAT master 交互，双方通过三重缓冲（SwapList）交接数据。

---

## 3. 三重缓冲 & Wrapper：三者的职责分工

### 3.1 SwapList / SwapNode（`common.h` / `common.cpp`）

- `SwapList` = 3 个 `SwapNode` 组成的环，`nodePtr` 指向当前节点。
- 每个 `SwapNode` 里有一整帧 PDO 内存（长度 = `domainSize`）。
- 重要操作：
  ```cpp
  void advanceNodePtr();          // nodePtr = nodePtr->next
  void copyTo(domainPtr,size);   // 把 nodePtr->previous 的帧拷到 EtherCAT 域
  void copyFrom(domainPtr,size); // 把域里的帧拷到 nodePtr->next, 再把 nodePtr 移到 next
  ```

### 3.2 DataWrapper（`common.h`）

对一个方向的 PDO 数据（Rx or Tx）包装为“结构体视图”：

```cpp
template<typename Data>
class DataWrapper {
public:
    int offset;      // 在 domain 内的字节偏移
    SwapList* swap;  // 所属域的三缓

    Data* operator->(){   // 当前节点
        return swap ? (Data*)(swap->nodePtr.load()->memPtr + offset) : data;
    }
    Data* previous(){     // 上一节点
        return swap ? (Data*)(swap->nodePtr.load()->previous->memPtr + offset) : data;
    }
    Data* next(){         // 下一节点
        return swap ? (Data*)(swap->nodePtr.load()->next->memPtr + offset) : data;
    }
};
```

### 3.3 WrapperPair（`common.h`）

每个电机对应一个 `WrapperPair<DriverRxData, DriverTxData, MotorParameters>`：

- 内部包含：
  - `DataWrapper<DriverRxData> rx;`  → 写入命令的视图
  - `DataWrapper<DriverTxData> tx;`  → 读取反馈的视图
  - `MotorParameters parameters;` → 极性、齿比、额定电流等
  - `busCode/order/domain/slave/alias` 等信息
- 初始化：
  ```cpp
  init(..., rxOffset, txOffset, ...);   // 记住 offset & 元数据
  config(..., rxSwap, txSwap);          // 绑定到具体 SwapList，加载参数
  ```

所以两者关系可以总结为：

- `SwapList` 管**原始三重缓冲**（3 个盒子轮着用）；
- `DataWrapper` 把 “SwapList + offset” 映射成一个结构体视图（`DriverRxData/DriverTxData`）；
- `WrapperPair` 把这两种视图 + 参数/元数据组合成“一整台电机”的包装；
- 上层只用 `drivers[i].rx->xxx` 和 `drivers[i].tx->xxx`，不用关心偏移和拷贝。

---

## 4. 一帧数据的完整往返流程

假设：

- 你已经跑过 `sdk.init`，ECAT 线程在 `run()` 后的 `rxtx` 中循环；

### 4.1 写指令：`single_motor_demo.cpp` → `DriverSDK::setMotorTarget`

1. 应用层循环：

```cpp
while (true){
    sdk.setMotorTarget(targets);
    sdk.getMotorActual(actuals);
    ...
}
```

2. `DriverSDK::setMotorTarget`（`loong_driver_sdk.cpp`）

- 对每个 ECAT 电机 i（`drivers[i].busCode == 0`）：

  ```cpp
  // 先把 pos/vel/tor 从物理单位转为设备内部编码，写到 Rx 当前节点
  drivers[i].rx->TargetPosition = ...;
  drivers[i].rx->TargetVelocity = ...;
  ...
  if (operatingMode[i] == 10) {       // CST 模式
      int torqueRaw = ...;            // per-mille of rated current/torque (e.g. 1000 = 100% rated)
      drivers[i].rx->MaxTorque    = maxCurrent[i];
      drivers[i].rx->TargetTorque = torqueRaw;
      printf("[DBG] 6071(raw)=%d\n", torqueRaw);
  }
  drivers[i].enabled = data[i].enabled;
  ```

- 然后做 CiA-402 状态机（使能驱动），每次写完关键 `ControlWord/Mode` 后调用 `advanceNodePtr()`：

  ```cpp
  drivers[i].rx->Mode = operatingMode[i];
  drivers[i].rx->ControlWord = 0x07;
  ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
  // 多次重复，确保多帧写入
  ```

此时：

- 电机 i 的这一帧命令已经写进了其所在域的 Rx 三重缓冲，并通过多次 `advanceNodePtr()` 至少有一帧落在 `nodePtr->previous` 上，准备给 EtherCAT 线程发。

### 4.2 发 PDO：`ECAT::rxtx`（发送部分）

在 rxtx 线程中，每个周期：

```cpp
for (int d = 0; d < domainCount; ++d){
    if (ecat->rxPDOSwaps[d] != nullptr && count % ecat->domainDivision[d] == 0){
        ecat->rxPDOSwaps[d]->copyTo(ecat->domainPtrs[d], ecat->domainSizes[d]);
        ecrt_domain_queue(ecat->domains[d]);
    }
}
ecrt_master_send(ecat->master);
```

- `domainDivision[d] == 1` → 每个周期都执行。
- `copyTo` 使用 `nodePtr->previous`：
  ```cpp
  memcpy(domainPtr, nodePtr.load()->previous->memPtr, domainSize);
  ```
  → 把上一帧稳定的数据拷到 EtherCAT 域，然后发到从站。

### 4.3 收 PDO：`ECAT::rxtx`（接收部分）

仍在 rxtx 线程：

```cpp
ecrt_master_receive(ecat->master);
...
for (int d = 0; d < domainCount; ++d){
    ecrt_domain_process(ecat->domains[d]);
    ecrt_domain_state(ecat->domains[d], &domainStates[d]);
    if (domainStates[d].wc_state == EC_WC_COMPLETE){
        ecat->txPDOSwaps[d]->copyFrom(ecat->domainPtrs[d], ecat->domainSizes[d]);
        // 收到的 TxPDO 写入 Tx 三缓的 node->next，并前移 nodePtr
    }
}
```

- `copyFrom`：
  ```cpp
  SwapNode* node = nodePtr.load();
  memcpy(node->next->memPtr, domainPtr, domainSize);
  nodePtr.store(node->next);
  ```
- 现在，Tx 三缓的 `nodePtr` 指向“最新一帧反馈”的盒子。

### 4.4 读反馈：`DriverSDK::getMotorActual`

应用层随后调用：

```cpp
std::vector<motorActualStruct> actuals(motorCount);
sdk.getMotorActual(actuals);
```

在 `loong_driver_sdk.cpp` 中：

```cpp
int DriverSDK::getMotorActual(std::vector<motorActualStruct>& data){
    ...
    for (int i = 0; i < dofAll; ++i){
        if (drivers[i].busCode == 0){   // ECAT 驱动
            data[i].pos = ... drivers[i].tx->ActualPosition ...;
            data[i].vel = ... drivers[i].tx->ActualVelocity ...;
            data[i].tor = ... drivers[i].tx->ActualTorque ...;
            data[i].statusWord = drivers[i].tx->StatusWord;
            data[i].errorCode  = drivers[i].tx->ErrorCode;
        }
    }
}
```

- `drivers[i].tx->` 调用的是 `DataWrapper::operator->()`：
  ```cpp
  return (Data*)(swap->nodePtr.load()->memPtr + offset);
  ```
- `swap` 是该电机所在域的 Tx 三缓，`nodePtr` 已被 rxtx 线程更新到最新一帧。

因此，这里读到的就是“刚刚从 EtherCAT 域拷回来的那一帧 TxPDO”，再经过单位转换返回给应用。

---

## 5. 一句话总结整个流程

1. `single_motor_demo.cpp` 调 `sdk.init`：
   - `loong_driver_sdk.cpp` 读 `config.xml` → 算出有几个电机 → 创建 `drivers[]` → 创建 `ECAT` 对象。
   - `ecat.cpp` 在 `ECAT::config` 中：
     - 扫描从站、按 alias/索引匹配
     - 为每个域创建 EtherCAT domain
     - 注册每个电机的 Rx/Tx PDO，拿到 offset
     - 创建该域的 Rx/Tx 三重缓冲 `SwapList`
     - 把每个 `WrapperPair` 的 `rx/tx` 绑定到对应 SwapList+offset

2. 控制循环：
   - `setMotorTarget`：写 `drivers[i].rx->字段`（Rx 三缓当前节点）→ `advanceNodePtr()` 把这一帧滚到 previous。
   - `ECAT::rxtx`：`copyTo` 用 Rx 三缓 `previous` 节点拷到 EtherCAT 域 → `ecrt_master_send` 发往从站；`copyFrom` 把域里的 TxPDO 拷到 Tx 三缓 `next` 节点并前移，形成最新帧。
   - `getMotorActual`：读 `drivers[i].tx->字段`（Tx 三缓当前节点），得到最新一帧反馈，转换成物理单位返回。

3. 三重缓冲的作用：
   - 用 3 个节点轮着存放“上一帧/当前帧/下一帧”，让写线程（控制逻辑）和 EtherCAT 线程在不同节点上操作，始终通过“上一帧稳定数据”交接，避免撕裂。*** End Patch
