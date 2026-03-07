# ECAT 三重缓冲 & Wrapper 流程梳理

> 目标：用“文件 → 函数 → 调用顺序”的方式，把现在这套三重缓冲（`SwapList`）和 `WrapperPair` 在 EtherCAT 控制流程中的角色讲清楚，方便以后查。

---

## 1. 总体结构图

- 控制线程（应用层）：`single_motor_demo.cpp` → `DriverSDK::setMotorTarget` / `getMotorActual`
- EtherCAT 线程：`ECAT::rxtx`（`ecat.cpp`）
- 数据缓冲：每个 EtherCAT 域有一对三重缓冲
  - `rxPDOSwaps[domain]`：往 EtherCAT 发的 RxPDO 三缓
  - `txPDOSwaps[domain]`：从 EtherCAT 收的 TxPDO 三缓
- 数据视图：每个电机一个 `WrapperPair<DriverRxData, DriverTxData, MotorParameters>`，内部用 `DataWrapper` 把三缓映射成结构体字段。

可以粗略理解为：

- 三个“盒子”（`SwapNode`）组成一个环 = `SwapList`
- 控制线程往“当前盒子”写指令，然后把盒子往后转几格
- EtherCAT 线程从“上一格盒子”取指令发到总线，再把新反馈写到“下一格盒子”
- 这样双方不会抢同一块内存，始终在用不同的盒子交接“上一帧稳定数据”

---

## 2. 初始化阶段：从哪里开始创建三缓和 Wrapper？

### 2.1 应用入口：`single_motor_demo.cpp`

```cpp
DriverSDK::DriverSDK& sdk = DriverSDK::DriverSDK::instance();
sdk.init(configPath.c_str());
```

- 这里只是入口，真正干活在 `DriverSDK` 的实现里。

### 2.2 DriverSDK 层：`heima_driver_sdk.cpp`

```cpp
void DriverSDK::init(char const* xmlFile){
    if (imp.init(xmlFile) < 0) {
        printf("imp init failed\n");
        exit(-1);
    }
}
```

- 进入 `DriverSDK::impClass::init`：
  - `configXML = new ConfigXML(xmlFile);`
  - 读电机拓扑：`motorAlias()` → 计算 `dofAll`
  - 分配电机包装对象：
    ```cpp
    if (dofAll > 0) {
        drivers = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>[dofAll];
    }
    ```
  - 读 EtherCAT 拓扑：
    ```cpp
    ecatAlias2type   = configXML->alias2type("ECAT");
    ecatAlias2domain = configXML->alias2attribute("ECAT", "domain");
    ecatDomainDivision = configXML->domainDivision("ECAT");
    ```
  - 创建 EtherCAT 主站对象：
    ```cpp
    while (i < ecatAlias2type.size()) {
        ecats.emplace_back(i);  // 调用 ECAT::ECAT(order)
        printf("ecats[%d] created\n", i);
        i++;
    }
    ```

### 2.3 ECAT 构造 & 域创建：`ecat.cpp`

```cpp
ECAT::ECAT(int const order){
    ...
    alias2type   = ecatAlias2type[order];
    alias2domain = ecatAlias2domain[order];
    domainDivision = ecatDomainDivision[order];
    while (init() < 0) {
        clean();
    }
}
```

- `ECAT::init()`：
  - `master = ecrt_request_master(order);`
  - 打开 `/dev/EtherCAT{order}`
  - 为每个域准备数组（先置空）：
    ```cpp
    domains      = new ec_domain_t*[domainDivision.size()];
    domainPtrs   = new unsigned char*[domainDivision.size()];
    domainSizes  = new int[domainDivision.size()];
    rxPDOSwaps   = new SwapList*[domainDivision.size()];
    txPDOSwaps   = new SwapList*[domainDivision.size()];
    ```

### 2.4 ECAT::check：绑定“配置里的设备”到“总线上的从站”

```cpp
int ECAT::check(){
    ...
    ec_master_info_t masterInfo;
    ecrt_master(master, &masterInfo);
    ...
    alias2slave.clear();
    std::vector<int> configAliases = ...;  // 从 alias2type 里读出配置的 alias

    int i = 0;
    while (i < masterInfo.slave_count) {
        ecrt_master_get_slave(master, i, &slaveInfo);
        ...
        // 从 SDO 读 Alias（或者依据 index 覆盖）
        int alias = readAlias(...);
        if (ecatMatchByIndex && i < configAliases.size()) {
            int cfgAlias = configAliases[i];
            printf(", override alias %d->%d (by index)", alias, cfgAlias);
            alias = cfgAlias;
        } else if (alias == 0 && alias2type.size() == 1) {
            alias = alias2type.begin()->first;
        }
        printf(", category %s, alias %d\n", category.c_str(), alias);

        auto itr = alias2type.find(alias);
        if (itr == alias2type.end()) {
            printf("\tdevice with the alias not found/enabled in xml\n");
            i++;
            continue;
        }
        ...
        alias2slave.insert(std::make_pair(alias, i)); // alias → 从站 index
        i++;
    }
    ...
}
```

- 这里决定“配置里的 alias=1/2”对应总线上的哪个从站 index。
- 如果驱动 alias=0，也可以通过 `ecatMatchByIndex` 按从站顺序强行匹配到 alias=1/2。

### 2.5 ECAT::config：注册 PDO + 创建三重缓冲 + 绑定 WrapperPair

1. 为每个域创建 EtherCAT domain：

```cpp
while (i < domainDivision.size()) {
    domains[i] = ecrt_master_create_domain(master);
    ...
    i++;
}
```

2. 为每个从站计算 PDO entry 和 offset：

```cpp
int alias = itr->first;
int slave = itr->second;
int domain = alias2domain.find(alias)->second;
std::string type = alias2type.find(alias)->second;
...
printf("master %d, domain %d, slave %d, alias %d, category %s, type %s\n", ...);

// 根据 xml 的 RxPDOs/TxPDOs 生成 pdoEntries / pdoInfos
// 注册 RxPDO/TxPDO 到对应 domain，拿到 offset：
int rxPDOOffset = ecrt_slave_config_reg_pdo_entry(...);
...
int txPDOOffset = ecrt_slave_config_reg_pdo_entry(...);
printf("\trxPDOOffset %d, txPDOOffset %d\n", rxPDOOffset, txPDOOffset);

// 创建 SDO/REG handler
ec_sdo_request_t* sdoHandler = ecrt_slave_config_create_sdo_request(...);
ec_reg_request_t* regHandler = ecrt_slave_config_create_reg_request(...);

// 初始化 WrapperPair：记录 bus/order/domain/slave/alias/type 和 offset
drivers[alias - 1].init(
    "ECAT", 0, order, domain, slave, alias, type,
    rxPDOOffset, txPDOOffset,
    sdoHandler, regHandler);
```

3. `ecrt_master_activate` 后，为每个域创建三缓 + 把 WrapperPair 绑定到对应 SwapList 上：

```cpp
domainPtrs[i]  = ecrt_domain_data(domains[i]);
domainSizes[i] = ecrt_domain_size(domains[i]);
printf("master %d, domain %d, domainPtr %ld, domainSize %d\n", ...);

// 为该域创建 Rx/Tx 三重缓冲
rxPDOSwaps[i] = new SwapList(domainSizes[i]);
txPDOSwaps[i] = new SwapList(domainSizes[i]);

// 遍历所有驱动，把对应 alias 放到这个域的 SwapList 上
int j = 0;
while (j < dofAll) {
    switch (drivers[j].config("ECAT", order, i, rxPDOSwaps[i], txPDOSwaps[i])) {
    case 2: // 还没 init
        drivers[j].tx->StatusWord = 0xffff;
        break;
    case 1: // 不属于这个域
        break;
    case 0: // 正常绑定 + 读参数
        break;
    case -1:
        printf("\tdrivers[%d] config failed\n", j);
        return -1;
    }
    j++;
}
```

到这一步为止：

- 每个域都有各自的 `rxPDOSwaps[i]` / `txPDOSwaps[i]`（三重缓冲，大小 = domainSize）。
- 每个电机（`drivers[k]`）知道：
  - 它属于哪个 master/order/domain/slave/alias
  - 它的 RxPDO 在这个域的字节偏移 `rxOffset`
  - 它的 TxPDO 在这个域的字节偏移 `txOffset`
  - Rx/Tx DataWrapper 绑定到了当前域对应的 SwapList 上

---

## 3. 三重缓冲（SwapList）和 Wrapper 的设计

### 3.1 SwapList & SwapNode：三重缓冲本体（`common.h / common.cpp`）

```cpp
class SwapNode {
public:
    unsigned char* memPtr;
    SwapNode* previous, * next;
    SwapNode(int size);
    ~SwapNode();
};

class SwapList {
public:
    std::atomic<SwapNode*> nodePtr;
    SwapList(int const size);
    void advanceNodePtr();
    void copyTo(unsigned char* domainPtr, int const domainSize);
    void copyFrom(unsigned char const* domainPtr, int const domainSize);
    ~SwapList();
};
```

实现（简化）：

```cpp
SwapList::SwapList(int size) {
    nodePtr.store(new SwapNode(size));    // 第一个节点
    SwapNode* current = nodePtr.load();
    // 再创建两个节点，串成环（3 个节点循环）
    for (int i = 1; i < 3; ++i) {
        SwapNode* node = new SwapNode(size);
        node->previous = current;
        current->next  = node;
        current = node;
    }
    nodePtr.load()->previous = current;
    current->next            = nodePtr.load();
}

void SwapList::advanceNodePtr(){
    nodePtr.store(nodePtr.load()->next);
}

void SwapList::copyTo(unsigned char* domainPtr, int domainSize){
    memcpy(domainPtr, nodePtr.load()->previous->memPtr, domainSize);
}

void SwapList::copyFrom(unsigned char const* domainPtr, int domainSize){
    SwapNode* node = nodePtr.load();
    memcpy(node->next->memPtr, domainPtr, domainSize);
    nodePtr.store(node->next);
}
```

总结：

- 有 3 个 `SwapNode`，每个有完整的一帧 PDO 内存（domainSize 字节）。
- `advanceNodePtr()`：当前节点往下走一格。
- `copyTo()`：把 **nodePtr->previous** 的那一帧拷到 EtherCAT 域（发出去）。
- `copyFrom()`：把 EtherCAT 域拷到 **nodePtr->next**，然后当前节点前移到 next。

### 3.2 DataWrapper & WrapperPair：给三缓加“结构体视图”

`DataWrapper`（common.h）简化版：

```cpp
template<typename Data>
class DataWrapper {
public:
    Data* data;      // 备用，不挂三缓时用
    int offset;      // 在 domain 内的字节偏移
    SwapList* swap;  // 对应域的 SwapList

    void init(int const offset){ this->offset = offset; }
    void config(SwapList* const swap){ this->swap = swap; }

    Data* operator->(){           // 当前节点
        if (swap) return (Data*)(swap->nodePtr.load()->memPtr + offset);
        return data;
    }
    Data* previous(){             // 上一节点
        if (swap) return (Data*)(swap->nodePtr.load()->previous->memPtr + offset);
        return data;
    }
    Data* next(){                 // 下一节点
        if (swap) return (Data*)(swap->nodePtr.load()->next->memPtr + offset);
        return data;
    }
};
```

`WrapperPair`（每个电机一个）：

```cpp
template<typename RxData, typename TxData, typename Parameters>
class WrapperPair {
public:
    int busCode, order, domain, slave, alias, enabled;
    std::string bus, type;
    DataWrapper<RxData> rx;
    DataWrapper<TxData> tx;
    ec_sdo_request_t* sdoHandler;
    ec_reg_request_t* regHandler;
    Parameters parameters;

    int init(..., int rxOffset, int txOffset, ...){
        this->order  = order;
        this->domain = domain;
        this->slave  = slave;
        this->alias  = alias;
        this->bus    = bus;
        this->type   = type;
        rx.init(rxOffset);
        tx.init(txOffset);
        this->sdoHandler = sdoHandler;
        this->regHandler = regHandler;
        return 0;
    }

    int config(std::string const& bus, int const order, int const domain,
               SwapList* const rxSwap, SwapList* const txSwap){
        if (this->order == -1) return 2;        // 未 init
        if (this->bus != bus || this->order != order || this->domain != domain) return 1;

        rx.config(rxSwap);
        tx.config(txSwap);

        if (parameters.load(bus, alias, type, sdoHandler) < 0){
            printf("loading parameters failed for %s slave %d:%d with alias %d\n", ...);
            return -1;
        }
        return 0;
    }
};
```

作用：

- 每个 `drivers[i]`（WrapperPair）知道自己属于哪个 master/domain/slave/alias。
- `rx/tx` 两个 `DataWrapper` 帮你把三缓里的原始字节映射成 `DriverRxData`/`DriverTxData` 结构体字段。
- 上层代码只需要写 `drivers[i].rx->TargetTorque`，`getMotorActual` 只需要读 `drivers[i].tx->ActualPosition`，不用自己管偏移和 memcpy。

### 3.3 结合当前 `config.xml`：一个 domain 的内存长什么样？`WrapperPair` 到底“指向哪里”？

先抓住一个事实：**EtherCAT 的 domain 本质上就是一段连续的 `unsigned char[]`（process image）**。这段内存里同时包含：

- **输出区（RxPDO entries）**：主站发给从站的数据
- **输入区（TxPDO entries）**：从站回给主站的数据

在你的实现里，同一个 domain 会准备两套三重缓冲：

- `rxPDOSwaps[domain]`：缓存“我要发出去的输出区快照”
- `txPDOSwaps[domain]`：缓存“我刚收回来的输入区快照”

> 这就是你强调的点：**一个 domain 对应两个 `SwapList`**（一个 RX、一个 TX），它们大小都等于 `domainSize`，但用途相反。

#### 3.3.1 以你当前 `config.xml` 的 `MT_Device` 为例（16B RxPDO + 16B TxPDO）

你现在的 PDO 定义（`config.xml`）是：

- RxPDO：`Controlword`、`TargetPosition`、`TargetVelocity`、`TargetTorque`、`MaxTorque`、`Mode`、`RxPDOPadding`（`config.xml:9`）
- TxPDO：`StatusWord`、`ActualPosition`、`ActualVelocity`、`ActualTorque`、`ErrorCode`、`ModeDisplay`、`TxPDOPadding`（`config.xml:19`）

并且你的结构体是 `__attribute__((packed))`，字段偏移和 bit_length 正好一一对应：

- `DriverRxData`：16 字节（`common.h:65`）
- `DriverTxData`：16 字节（`common.h:84`）

在你“一个 domain 只放一台从站”的当前配置下（`config.xml:69`、`config.xml:71`），domain 的 process image 通常会是这样一段 32 字节（实际以 `ecrt_domain_size()`/`ecrt_slave_config_reg_pdo_entry()` 结果为准）：

```text
domainX 的 process image（示意：RxPDO 16B + TxPDO 16B）

[0 .. 15]  RxPDO（输出区，对应 DriverRxData，common.h:65）
  0-1   ControlWord     (0x6040)
  2-5   TargetPosition  (0x607A)
  6-9   TargetVelocity  (0x60FF)
 10-11  TargetTorque    (0x6071)
 12-13  MaxTorque       (0x6072)
 14     Mode            (0x6060)
 15     RxPDOPadding    (0x5FFE)

[16 .. 31] TxPDO（输入区，对应 DriverTxData，common.h:84）
 16-17  StatusWord      (0x6041)
 18-21  ActualPosition  (0x6064)
 22-25  ActualVelocity  (0x606C)
 26-27  ActualTorque    (0x6077)
 28-29  ErrorCode       (0x603F)
 30     ModeDisplay     (0x6061)
 31     TxPDOPadding    (0x5FFE)
```

#### 3.3.2 “映射成结构体字段”到底是怎么发生的？

`WrapperPair` 里最关键的其实不是一堆字段，而是这两件事：

1) **offset**：这台从站的 RxPDO/TxPDO 在 domain 内从第几个字节开始（由 `ecrt_slave_config_reg_pdo_entry()` 算出来）  
2) **swap 指针**：这台从站属于哪个 domain，于是 `rx` 绑定到 `rxPDOSwaps[domain]`、`tx` 绑定到 `txPDOSwaps[domain]`

当你在上层写：

- `drivers[i].rx->TargetTorque = ...;`

其实 `DataWrapper::operator->()`（`common.h:231`）做的是：

```cpp
(DriverRxData*)(rxPDOSwaps[domain]->nodePtr.load()->memPtr + rxPDOOffset)
```

也就是说：**把“某个盒子里的原始字节数组 + 起始偏移”强转为 `DriverRxData*`**，然后用 C/C++ 的结构体字段偏移去访问那几个字节。

读反馈同理：

- `drivers[i].tx->ActualPosition` 等价于把 `txPDOSwaps[domain]->nodePtr->memPtr + txPDOOffset` 强转为 `DriverTxData*` 再取字段。

这也是为什么你这套必须满足：`config.xml` 里 PDO 的 object 顺序/bit_length 和 `DriverRxData/DriverTxData` 的 packed 布局一致；否则“指针强转”就会读写错位。

#### 3.3.3 结合你当前的两台电机（两个 domain）

你现在把：

- alias=1 放到 domain=0（`config.xml:69`）
- alias=2 放到 domain=1（`config.xml:71`）

于是每个 domain 都是“1 台从站 + 1 套（Rx/Tx）三缓冲”，互相独立；`drivers[0]` 只会指向 domain0 的两套 SwapList，`drivers[1]` 只会指向 domain1 的两套 SwapList。

---

## 4. 运行时调用顺序：从哪个文件到哪个文件？

这里按一次循环来梳理（已经上电，402 状态机进入 OP 后）：

### 4.1 `single_motor_demo.cpp`（应用层）

```cpp
while (true) {
    sdk.setMotorTarget(targets);             // 写命令 → heima_driver_sdk.cpp
    std::vector<motorActualStruct> actuals(motorCount);
    sdk.getMotorActual(actuals);             // 读反馈 → heima_driver_sdk.cpp
    ...
}
```

### 4.2 写命令路径（控制线程）

1. `DriverSDK::setMotorTarget`（`heima_driver_sdk.cpp`）：

- 根据 `data[i]` 的 pos/vel/tor，把物理量转成设备内部单位，写 `drivers[i].rx->...`：
  ```cpp
  // 例：CST 模式下的扭矩
  int torqueRaw = static_cast<int>(drivers[i].parameters.polarity * torque);
  drivers[i].rx->MaxTorque    = maxCurrent[i];
  drivers[i].rx->TargetTorque = torqueRaw;
  printf("[DBG] tx-node %p 6071(raw)=%d\n",
         imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->rxPDOSwaps[...]->nodePtr.load()->previous->memPtr,
         torqueRaw);
  ```

- 然后根据当前 StatusWord 做 402 状态机的 ControlWord 流程，每写一次关键命令后调用 **`advanceNodePtr()`** 多次，让这一帧滚到“上一格盒子”：
  ```cpp
  drivers[i].rx->Mode = operatingMode[i];
  drivers[i].rx->ControlWord = 0x07;
  imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
  // 再重复几次，确保有多帧连续写入
  ```

2. 这一步完成后：

- 当前 SwapList 的 `nodePtr` 已经往前转了几格；你刚写的那帧数据现在位于 `nodePtr->previous`。
- 下一步就等 rxtx 线程来调用 `copyTo()`，把 `previous` 那一帧发到 EtherCAT 域。

### 4.3 发 PDO / 收 PDO（EtherCAT 线程）

1. `ECAT::rxtx`（`ecat.cpp` 中的静态线程函数）：

- 发送（把 Rx 三缓 → EtherCAT domain）：
  ```cpp
  // 每个域 i
  if (ecat->rxPDOSwaps[i] != nullptr && count % ecat->domainDivision[i] == 0) {
      ecat->rxPDOSwaps[i]->copyTo(ecat->domainPtrs[i], ecat->domainSizes[i]);
      ecrt_domain_queue(ecat->domains[i]);
  }
  ecrt_master_send(ecat->master);
  ```

  - `copyTo` 用的是 `nodePtr->previous`，也就是你在 `setMotorTarget + advanceNodePtr` 写好的那帧。
  - `ecrt_master_send` 把这一帧真正发到了从站。

- 接收（从 EtherCAT domain → Tx 三缓）：
  ```cpp
  ecrt_master_receive(ecat->master);
  ...
  if (domainStates[i].wc_state == EC_WC_COMPLETE) {
      ecat->txPDOSwaps[i]->copyFrom(ecat->domainPtrs[i], ecat->domainSizes[i]);
      ...
  }
  ```

  - `copyFrom` 把域里的数据写到 `txSwap.nodePtr->next->memPtr`，然后前移 nodePtr 到 next。
  - 此时 `txSwap.nodePtr` 指向的是“最新一帧反馈”的盒子。

### 4.4 读反馈路径（控制线程）

1. `DriverSDK::getMotorActual`（`heima_driver_sdk.cpp`）：

```cpp
if (drivers[i].busCode == 0) { // ECAT 驱动
    data[i].pos = 2.0 * Pi * drivers[i].parameters.polarity *
                  (drivers[i].tx->ActualPosition - drivers[i].parameters.countBias) / ...;
    data[i].vel = ...
    data[i].tor = ...
    data[i].statusWord = drivers[i].tx->StatusWord;
    data[i].errorCode  = drivers[i].tx->ErrorCode;
}
```

- `drivers[i].tx->` 访问的是 `DataWrapper::operator->()`：
  ```cpp
  return (Data*)(swap->nodePtr.load()->memPtr + offset);
  ```
- 由于 rxtx 刚刚在 `copyFrom` 里把 `nodePtr` 移到了最新一帧，当前节点就是最新反馈。

2. `single_motor_demo.cpp` 拿到 `actuals` 后把它打印出来，用来观察 pos/vel/tor/statusWord 是否工作正常。

---

## 5. 总结：一句话记忆版

- **初始化**：
  - `single_motor_demo.cpp` → `DriverSDK::init` → `imp.init`（`heima_driver_sdk.cpp`）→ `ECAT::ECAT/init/check/config`（`ecat.cpp`）。
  - `ECAT::config` 创建每个域的 `SwapList`（Rx/Tx 三重缓冲），并把每个电机的 `WrapperPair.rx/tx` 绑定到对应 SwapList + offset。

- **写指令（控制线程）**：
  - `single_motor_demo` → `DriverSDK::setMotorTarget`：写 `drivers[i].rx->...`（= Rx 三缓当前节点）→ `advanceNodePtr()` 把这一帧滚到 previous，等 rxtx 用。

- **发包 / 收包（EtherCAT 线程）**：
  - `ECAT::rxtx`：`copyTo` 用 Rx 三缓 `previous` 节点的帧发到 EtherCAT 域；`copyFrom` 把域里回来的一帧写到 Tx 三缓 `next` 节点并前移。

- **读反馈（控制线程）**：
  - `DriverSDK::getMotorActual`：读 `drivers[i].tx->...`（= Tx 三缓当前节点 = 最新一帧反馈），计算成物理量返回。

这一整套设计就是：

> 三个盒子（SwapList），一边写一边转，发送总用上一帧，接收总落下一帧，用 WrapperPair 把这几块内存包装成“每个电机的一组字段”，控制线程和 EtherCAT 线程互不打架，各拿到一帧稳定数据。
