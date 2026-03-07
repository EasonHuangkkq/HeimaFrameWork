# EtherCAT + 三重缓冲数据流（从初始化到下发）

## 1. 初始化到运行线程
1) `single_motor_demo.cpp`  
   - 设定模式向量（默认 0x0A CST），调用 `DriverSDK::init("config.xml")`。
2) `DriverSDK::impClass::init`（`loong_driver_sdk.cpp`）  
   - 解析 `config.xml`，构建 alias/类型/域分配、关节 DOF 等。  
   - 为每个 EtherCAT master 创建 `ECAT` 对象并调用 `check()`（链路、从站信息）→ `config()` → `run()`。
3) `ECAT::config`（`ecat.cpp`）  
   - `ecrt_master_create_domain` 创建域；按 XML 生成 PDO 描述，调用 `ecrt_slave_config_pdos`、`ecrt_domain_reg_pdo_entry_list`，得到域大小/偏移。  
   - 为每个域创建三重缓冲 `SwapList`（rx/tx 各一），把偏移、SwapList 绑定到 `drivers[*]`。  
   - `ecrt_master_activate` 取 `domainPtr` / `domainSize`。  
4) `ECAT::run` 启动实时线程 `rxtx`：负责 copyTo/queue/send/receive/copyFrom 和 DC 同步。

## 2. 每周期时序（三重缓冲指针动作）
写线程（应用，`setMotorTarget`）  
- 把目标写入 `drivers[i].rx` → 实质写到 `rxSwap->nodePtr->memPtr`（当前节点）。  
- 状态机推进（0x06→0x07→0x0F，进 OP 后 ControlWord 保持 0x1F）。  
- 每周期写 `MaxTorque= maxCurrent`。CST：`TargetTorque` 为额定电流/力矩的千分之一（per-mille），限幅 ±`maxCurrent`。  
- 调用 `imp.ecatUpdate()`：对每个域执行一次 `rxSwap->advanceNodePtr()`，因此 **nodePtr->previous 指向刚写好的节点**。

发送线程（`rxtx`，`ecat.cpp`）  
- 若域到周期：`copyTo` 用 `rxSwap->nodePtr->previous` 复制到 `domainPtr`，`ecrt_domain_queue`，`ecrt_master_send`。  
- 接收后 `ecrt_master_receive` / `ecrt_domain_process`，`txSwap->copyFrom(domainPtr, size)` 并 `advanceNodePtr()`，让应用读到上一帧快照。

读线程（应用，`getMotorActual` 等）  
- 通过 `drivers[i].tx` 读 `txSwap->nodePtr->previous`（上一帧的反馈）。

指针小结：写线程 advance 一次（写完）；rxtx 接收后 advance 一次（读完）。中间不再多余 advance，否则会跳过最新数据。

## 3. 关键 PDO 偏移（相对域首地址，MT_Device 0x1600/0x1A00）
- 0: 0x6040 ControlWord (U16)
- 2: 0x607A TargetPosition (S32)
- 6: 0x60FF TargetVelocity (S32)
- 10: 0x6071 TargetTorque (S16) ← 力矩/电流指令（千分之一额定）
- 12: 0x6072 MaxTorque (U16)
- 14: 0x6060 Mode (S8)
- 16: 0x6041 StatusWord (U16) 后续 TxPDO 按映射顺序排列

## 4. 验证手册（从总线视角闭环）
1) 运行示例：`sudo ./build/single_motor_demo 300 10 config.xml`  
2) 看工作计数：`sudo /usr/local/bin/ethercat domains` → `WorkingCounter 3/3`（单从站 32 字节域）。  
3) 读寄存器：  
   - `sudo /usr/local/bin/ethercat upload -p0 0x6061 0` 应为 0x0A（CST）  
   - `sudo /usr/local/bin/ethercat upload -p0 0x6071 0` 应等于命令（如 300=30% 额定）  
   - `sudo /usr/local/bin/ethercat upload -p0 0x6072 0` 应等于限幅（如 1000=100% 额定）  
4) 如需看三缓冲→域：在 `copyTo` 前打印 `*(short*)(rxSwap->nodePtr->previous->memPtr + 10)` 与 `*(short*)(domainPtr + 10)`，应一致；再用 `upload 0x6071` 对照。

## 5. 主要代码锚点
- 写入/状态机/限流：`loong_driver_sdk.cpp` (`setMotorTarget`)  
- 三重缓冲定义：`common.h` (`SwapList`, `DataWrapper`)  
- EtherCAT 线程与 copyTo/copyFrom：`ecat.cpp` (`ECAT::rxtx`)  
- 示例入口：`single_motor_demo.cpp`

## 6. 常见坑（已修正）
- 在 OP (0x0037) 额外 advance 会跳过最新节点 → 现已移除。  
- CST 模式 `TargetTorque` 与 `TorqueOffset` 为同字段，写完再清零会丢指令 → 现已保留写入值。  
- 0x6072 默认为 0 时驱动忽略力矩 → 每周期写 `MaxTorque=maxCurrent`。

## 7. 跨文件依赖与命名空间共享
- 命名空间：所有相关类/全局变量都在 `namespace DriverSDK` 中。
- 声明/实现分布：
  - 类声明在头文件，定义在对应 cpp：`ecat.h`/`ecat.cpp`，`loong_driver_sdk.h`/`loong_driver_sdk.cpp`，`config_xml.h`/`config_xml.cpp`，`common.h`/`common.cpp`。
  - 示例入口：`single_motor_demo.cpp` 包含 `loong_driver_sdk.h`。
- 全局指针/表的定义与 extern：
  - 定义：`loong_driver_sdk.cpp` 顶部定义 `ConfigXML* configXML;` 及 `drivers`、`ecatAlias2type` 等全局。
  - 使用：`ecat.cpp` 顶部用 `extern ConfigXML* configXML;` 等引入这些变量，依赖同一命名空间。
- 依赖链概览：
  - 应用（demo）→ `loong_driver_sdk.h/.cpp`（创建 DriverSDK、管理总线与全局表）→ 包含 `ecat.h`、`config_xml.h`。
  - `ecat.cpp` 包含 `ecat.h`、`config_xml.h`，通过 `extern` 访问 `configXML`、`drivers` 等（定义在 `loong_driver_sdk.cpp`）。
  - `config_xml.cpp` 实现 XML 解析，头文件供两侧包含。
  - `common.h/.cpp`：定义/实现三重缓冲（SwapList/DataWrapper/WrapperPair）、PDO 数据结构（DriverRxData/DriverTxData 等）和工具函数，供 `loong_driver_sdk.cpp`、`ecat.cpp` 共同包含；自身只依赖 `config_xml.h`（用于 `MotorParameters::load`）。

### 7.1 逐文件角色与包含关系（极细）
- `single_motor_demo.cpp`
  - 包含：`loong_driver_sdk.h`
  - 调用：`DriverSDK::instance()` 获取单例；`setMode`/`init`/`setMotorTarget`/`getMotorActual` 等。
- `loong_driver_sdk.h`
  - 声明：`DriverSDK` 类接口、`motorSDOClass`/`motorREGClass`、各数据结构（motorTargetStruct 等）。
  - 不定义全局变量，仅声明接口。
- `loong_driver_sdk.cpp`
  - 定义（命名空间内的真正存储）：`ConfigXML* configXML;`、`drivers`、`ecatAlias2type`、`maxCurrent` 等所有跨文件全局。
  - 包含：`loong_driver_sdk.h`、`config_xml.h`、`ecat.h`、`common.h`、`rs232.h`、`rs485.h`、`can.h`。
  - 构造 `DriverSDK` 单例，创建 `ECAT` 对象并调用 `check/config/run`。
  - 内部类 `impClass` 管理总线和线程调度。
  - 提供 `fillSDO`/`sendMotorSDORequest`/`recvMotorSDOResponse` 等接口；`motorSDOClass` 构造/析构在此定义。
- `ecat.h`
  - 声明：`class ECAT`（构造、`init`/`config`/`run` 等）、内部成员（domains、SwapList 指针等）。
- `ecat.cpp`
  - 定义：`ECAT` 全部方法、`rxtx` 线程。
  - 包含：`config_xml.h`、`rs485.h`、`ecat.h`、`common.h`。
  - `extern` 引入：`ConfigXML* configXML;`、`ecatAlias2type`、`drivers` 等（定义在 `loong_driver_sdk.cpp`）。
- `config_xml.h`
  - 声明：`class ConfigXML`（XML 解析接口、属性/映射获取）。
- `config_xml.cpp`
  - 定义：`ConfigXML` 构造/方法实现，加载 `config.xml`，被 `loong_driver_sdk.cpp`、`ecat.cpp` 使用。
- `common.h`
  - 声明：基础工具/数据结构（SwapList/WrapperPair、DriverRxData/TxData、SDOMsg/REGMsg、转换函数）。
  - 所有定义都在 `namespace DriverSDK`。
- `common.cpp`
  - 定义：`single2half`/`half2single`、`SwapList`、`MotorParameters::load` 等；包含 `common.h`、`config_xml.h`，不依赖业务文件。

### 7.2 实例化与符号解析从何处开始
- **configXML/全局表的定义点**：`loong_driver_sdk.cpp` 顶部（命名空间内）。这是唯一的存储。
- **使用点**：`ecat.cpp` 顶部 `extern` 声明，同一命名空间保证链接到上述定义。
- **ECAT 类声明/实现**：声明在 `ecat.h`，实现于 `ecat.cpp`，被 `loong_driver_sdk.cpp` 包含并实例化。
- **三缓冲/数据结构的声明与实现**：声明在 `common.h`，实现于 `common.cpp`，被 `loong_driver_sdk.cpp`、`ecat.cpp` 包含。
- **入口调用链**：`single_motor_demo.cpp` → `DriverSDK::init` → new `ConfigXML` → 填全局表 → 构造/配置 `ECAT` → 启动 `ECAT::rxtx`。
