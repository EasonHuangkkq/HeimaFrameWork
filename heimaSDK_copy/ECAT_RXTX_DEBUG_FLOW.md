# ECAT/DriverSDK 调试链路（从 config.xml 到电机转动）

本文把 **config.xml 读取 → EtherCAT 配置 → 三重缓冲 → rxtx 线程 → 电机转动** 的链路按函数顺序串起来，并标注关键变量的创建位置，方便逐函数打断点。

> 相关文件：
> - `heimaSDK/config_xml.cpp`
> - `heimaSDK/heima_driver_sdk.cpp`
> - `heimaSDK/ecat.cpp`
> - `heimaSDK/common.h` / `heimaSDK/common.cpp`
> - `heimaSDK/main/driver/task_driver.cpp`
> - `heimaSDK/ECAT_TRIPLE_BUFFER_FLOW.md`

---

## 1. 总览链路（函数顺序）

**入口 → SDK → ECAT → rxtx → 电机**：

1. `single_motor_demo.cpp` 或 `task_driver.cpp`
   - 调用 `DriverSDK::instance()`
   - 调用 `DriverSDK::init(config.xml)`

2. `DriverSDK::init()` (`heima_driver_sdk.cpp`)
   - 进入 `DriverSDK::impClass::init(xmlFile)`

3. `impClass::init()` (`heima_driver_sdk.cpp`)
   - `configXML = new ConfigXML(xmlFile)` → 读取 XML
   - `configXML->motorAlias()` → 计算 `dofAll` 并 `drivers = new WrapperPair[...]`
   - `configXML->alias2type("ECAT")` / `alias2attribute("ECAT","domain")`
   - `configXML->domainDivision("ECAT")`
   - `ecats.emplace_back(i)` 创建 ECAT 主站实例（每个 master 一次）

4. `ECAT::ECAT(order)` (`ecat.cpp`)
   - 读取全局 `ecatAlias2type / ecatAlias2domain / ecatDomainDivision`
   - 调用 `ECAT::init()`

5. `ECAT::init()` (`ecat.cpp`)
   - `master = ecrt_request_master(order)`
   - 分配 `domains / domainPtrs / domainSizes / rxPDOSwaps / txPDOSwaps` 数组

6. `ECAT::check()` (`ecat.cpp`)
   - 扫描从站，确定 alias → slave index
   - 写入 `alias2slave`

7. `ECAT::config()` (`ecat.cpp`)
   - `ecrt_master_create_domain()` 创建每个 domain
   - 注册 PDO entry，拿到 `rxOffset/txOffset`
   - `drivers[alias-1].init(...)` 记录 offset
   - `ecrt_master_activate()`
   - `domainPtrs[i] = ecrt_domain_data()`
   - `domainSizes[i] = ecrt_domain_size()`
   - `rxPDOSwaps[i] = new SwapList(domainSize)`
   - `txPDOSwaps[i] = new SwapList(domainSize)`
   - `drivers[j].config(..., rxPDOSwaps[i], txPDOSwaps[i])`

8. `ECAT::run()` (`ecat.cpp`)
   - `pthread_create(&pth, rxtx, this)` 启动实时线程

9. 应用控制循环（`task_driver.cpp` 或 demo）
   - `DriverSDK::setMotorTarget()` 写入三缓
   - `DriverSDK::getMotorActual()` 读取三缓

10. `ECAT::rxtx()` (`ecat.cpp`)
   - 周期性 `copyTo` 发送 + `copyFrom` 接收
   - 电机收到 PDO → 执行动作 → 反馈回主站

---

## 2. 关键变量“在哪创建”

### 全局/SDK 层（`heima_driver_sdk.cpp`）
- `ConfigXML* configXML`：在 `impClass::init()` 里 new
- `drivers`：`new WrapperPair<...>[dofAll]`
- `ecatAlias2type / ecatAlias2domain / ecatDomainDivision`：由 `ConfigXML` 解析写入
- `ecats`：`ecats.emplace_back(i)`

### ECAT 层（`ecat.cpp`）
- `master`：`ECAT::init()` 里 `ecrt_request_master(order)`
- `domains / domainPtrs / domainSizes`：`ECAT::init()` 分配数组；`ECAT::config()` 填充
- `rxPDOSwaps / txPDOSwaps`：`ECAT::config()` 中 `new SwapList(domainSize)`
- `alias2slave`：`ECAT::check()` 中建立

### 三缓冲层（`common.h/.cpp`）
- `SwapList`：`SwapList(size)` 构造 3 个 `SwapNode` 环
- `DataWrapper`：`drivers[i].rx/tx`，持有 offset + SwapList 指针

---

## 3. rxtx 线程在做什么（核心循环）

`ECAT::rxtx(void* arg)`（`ecat.cpp`）

**A. SDO 状态机**
- 从 `sdoRequestQueue` 取 `SDOMsg`
- 按 `state 0 → 1 → 2 → 3` 流程处理读/写
- 完成后放入 `sdoResponseQueue`

**B. 周期性发送（Tx PDO）**
- `count++`
- 若 `count % domainDivision[i] == 0`：
  - `rxPDOSwaps[i]->copyTo(domainPtrs[i], domainSizes[i])`
  - `ecrt_domain_queue(domains[i])`
- `ecrt_master_send(master)`

**C. 周期睡眠与 DC 同步**
- `wakeupTime` 控制周期
- `nanosleep()` 降 CPU 占用
- `ecrt_master_sync_reference_clock_to` / `ecrt_master_sync_slave_clocks`

**D. 接收与回写（Rx PDO）**
- `ecrt_master_receive(master)`
- `ecrt_domain_process(domains[i])`
- 若 `wc_state == EC_WC_COMPLETE`：
  - `txPDOSwaps[i]->copyFrom(domainPtrs[i], domainSizes[i])`

---

## 4. DriverSDK 写入/读取三缓的时序

- **控制方向**：
  - `DriverSDK::setMotorTarget()` 写 `drivers[i].rx->TargetTorque/...`
  - `impClass::ecatUpdate()` 里 `rxSwap->advanceNodePtr()`
  - `rxtx` 从 `previous` 取稳定帧 `copyTo()`

- **反馈方向**：
  - `rxtx` 收到数据 `copyFrom()` → 写入 `next` 并 advance
  - `DriverSDK::getMotorActual()` 读 `drivers[i].tx`（稳定帧）

---

## 5. C++ 写法原因 & 如果不这么写会怎样

- `void* ECAT::rxtx(void* arg)`：
  - 因为 `pthread_create` 只能接 C 风格函数指针，所以用 `void*` 转回 `ECAT*`
  - 换成 `std::thread` 可以用成员函数/lambda，但这里更偏 RT 风格

- `int workingCounters[domainCount]` / `ec_domain_state_t domainStates[domainCount]`：
  - 这是 **VLA**（GCC 允许，标准 C++ 不支持）
  - 更规范：`std::vector<int> workingCounters(domainCount);`

- 不用 `SwapList` 直接读写 `domainPtr`：
  - 应用线程与实时线程会数据竞争，可能读到“半帧”

- 不做 `domainDivision` 分频：
  - 所有域每周期发送，带宽/CPU 增大，容易掉周期

- 不用 `nanosleep` 控时：
  - 忙等会占满 CPU，实时性下降

---

## 6. 快速断点建议

- XML 解析入口：`ConfigXML::ConfigXML` (`config_xml.cpp`)
- ECAT 资源创建：`ECAT::init` (`ecat.cpp`)
- PDO 注册：`ECAT::config` (`ecat.cpp`)
- 三缓绑定：`drivers[j].config(...)` (`ecat.cpp`)
- 发送点：`rxPDOSwaps[i]->copyTo(...)` (`ecat.cpp`)
- 接收点：`txPDOSwaps[i]->copyFrom(...)` (`ecat.cpp`)
- 控制写入点：`DriverSDK::setMotorTarget` (`heima_driver_sdk.cpp`)
- 反馈读取点：`DriverSDK::getMotorActual` (`heima_driver_sdk.cpp`)

---

## 7. 关联文档

- `heimaSDK/ECAT_TRIPLE_BUFFER_FLOW.md`
- `heimaSDK/DATAFLOW.md`
- `heimaSDK/ARCHITECTURE_ANALYSIS.md`

