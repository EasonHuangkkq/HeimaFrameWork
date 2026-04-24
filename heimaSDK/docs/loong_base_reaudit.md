# loong_base-main 复审审计报告

> 复审目标：核对 `heimaSDK/docs/loong_base_audit.md` 中的问题是否被源码支持，并补充复审时发现的额外问题。  
> 复审范围：`loong_base-main` 当前工作区源码与当前配置文件。  
> 复审基线：本报告基于当前仓库状态，尤其基于 `config/driver.ini` 中的 **15 电机下肢配置** 与 `fingerDofs=0,0`。

---

## 结论摘要

这次复审的结论不是“原报告全错”，而是：

1. **确实存在真实缺陷**，其中最重要的是：
   - locomotion 掉线后 driver 仍持续下发最后一帧控制命令；
   - interface 的 UDP 接收线程析构逻辑存在真实竞态；
   - 右手手指拷贝 bug 在源码中真实存在，但在当前下肢配置中处于休眠状态。
2. **也存在被夸大的条目**，尤其是：
   - 共享内存“三缓冲必然撕裂”的表述过重；
   - interface 线程安全问题被描述成了控制链路级灾难，但代码里真正的问题更偏向**进程内并发读写**，不是“locomotion 直接读到半更新 `dc.cmd`”；
   - 相同优先级/同核绑定是调度风险，但不能直接从当前代码推出“必然导致 EtherCAT 超时”。
3. **复审额外发现了一个原报告未写出的实际 bug**：
   - `task_interface.cpp` 中对 `ocuFeed.posTarget[2]` / `ocuFeed.posPerson[2]` 的赋值存在越界写。

---

## 当前配置前提（很重要）

当前 `loong_base-main/config/driver.ini` 为：

```ini
drvNums=15
fingerDofs=0,0
idMap=12,13,14, 0,1,2,3,4,5, 6,7,8,9,10,11
```

这意味着本次复审是在 **下肢 15 电机、无手指自由度** 的配置上做判断。凡是涉及 31 关节全身配置、灵巧手、上肢联动的风险，都要区分：

- **源码里是否真有 bug**；
- **当前配置下是否会触发**。

---

## 复审结论表

| ID | 复审结论 | 说明 |
|---|---|---|
| C1 共享内存数据撕裂 | **部分成立，但原报告表述偏重** | 共享内存实现确实不够强健，但原报告对“必然混帧/多段 memcpy 更严重”的论证不准确 |
| C2 driver 掉线后继续下发旧命令 | **成立，且是当前最重要问题之一** | `Lost` 后没有进入安全输出路径，仍继续发送最后一帧 locomotion 控制 |
| C3 手指复制粘贴 bug | **成立，但当前配置休眠** | 源码 bug 真实存在；当前 `fingerDofs=0,0`，因此当前配置不触发 |
| C4 interface UDP 线程安全 | **部分成立，但原报告表述偏重** | 存在进程内数据竞争；但 `recvBuf` 并非并发覆盖，locomotion 也不是直接读半更新 `dc.cmd` |
| M1 `ocuFeedbackStruct` 写死 31 关节 | **部分成立** | 当前配置下不会越界，但未使用槽位可能发出垃圾数据 |
| M2 `sleep_until` 连续追赶 | **行为存在，但更像设计取舍/监测缺失** | 这是无超限检测的固定周期调度，不宜直接等同于严重逻辑 bug |
| M3 `ftok("/home", mark)` 脆弱性 | **有一定运维脆弱性，但原报告偏重** | 同机 IPC 仍可正常工作；跨设备复现不是这里的核心问题 |
| M4 全任务同优先级 | **成立，但要降格理解** | 是调度风险，不是从代码就能直接推出“driver 必被饿死” |
| M5 driver/loco 共用 CPU 2 | **成立，但影响被夸大** | 是实时性风险；但 SDK 的 EtherCAT 线程单独绑在 `driverSdk.cpu=1` |
| m1 字符串字面量死代码 | **成立但轻微** | 的确是无效语句 |
| m2 interface 析构竞态 | **成立，且是当前真实 bug** | 停线程标志失效，且 `delete` 时未等待线程结束 |
| m3 缺少 `pthread_attr_destroy` | **成立但轻微** | 一次性资源清理缺失 |
| m4 未使用常量 | **成立但轻微** | 仅代码整洁问题 |

---

## 一、确认成立的真实问题

### A1. locomotion 掉线后，driver 仍继续下发最后一帧控制命令

**文件**: [task_driver.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/driver/task_driver.cpp#L326-L419)

```cpp
if(dc.jntCtrlLoco.getFromShm()>-100){
    if(locoState!=Online){
        print("==loco上线！==");
        locoState=Online;
    }
}else{
    if(locoState==Online){
        print("==loco离线！==");
        locoState=Lost;
    }
}
jntCtrl.j=dc.jntCtrlLoco.j;
jntCtrl.w=dc.jntCtrlLoco.w;
jntCtrl.t=dc.jntCtrlLoco.t;
...
sdk.setMotorTarget(motTgts);
```

**复审结论**：成立，而且这是当前最值得优先修的控制链路问题。

**原因**：

1. `getFromShm() <= -100` 只会把 `locoState` 置成 `Lost`；
2. 后续代码**没有因为 `Lost` 改走安全分支**；
3. 反而继续把 `dc.jntCtrlLoco` 中最后一次成功接收到的控制量复制进 `jntCtrl`，并继续下发到 SDK；
4. `waitOP` 分支里的安全输出逻辑只在 `waitOP!=0` 时走，不是掉线保护。

**补充说明**：原报告写“100ms 后才检测离线”是对的，但还少说了一点：**即使离线已经被检测到，旧命令仍会继续被发送，不止 100ms。**

---

### A2. interface 析构时的线程退出逻辑失效，存在真实竞态

**文件**: [task_interface.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface.cpp#L231-L245), [task_interface.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface.cpp#L463-L466)

```cpp
void interfaceTaskClass::impClass::udpRcv(){
    Timing::sleepMs(20);
    bool udpRunning=1;
    ...
    while(udpRunning){
        udpRcvOcu(recvBuf);
        ...
    }
}

~interfaceTaskClass(){
    imp.udpRunning=0;
    delete &imp;
}
```

**复审结论**：成立。

**原因**：

1. `udpRcv()` 内部定义了一个**局部变量** `bool udpRunning=1;`；
2. 析构函数里写的是成员变量 `imp.udpRunning=0;`；
3. 两者不是同一个变量，因此析构时并不会让接收线程退出；
4. 线程是 `detach()` 的，析构时也没有 `join`；
5. `delete &imp` 后，后台线程仍可能继续访问 `recvBuf`、`dc`、成员 UDP server 等对象，形成 use-after-free 风险。

这条在源码层面非常明确，不需要依赖额外假设。

---

### A3. 额外发现：`ocuFeed.posTarget[2]` / `posPerson[2]` 存在越界写

**文件**: [task_interface.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface.cpp#L160-L169), [task_interface_data.h](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface_data.h#L160-L168)

`ocuFeedbackStruct` 定义：

```cpp
float posTarget[2];
float posPerson[2];
```

实际赋值：

```cpp
ocuFeed.posTarget[2]={0.0};
ocuFeed.posPerson[2]={0.0};
```

**复审结论**：这是原报告遗漏的真实 bug。

`[2]` 对于长度为 2 的数组来说已经越界。这里不是“语义不好”，而是**直接写到数组边界之外**。由于结构体是紧凑排列的，这会污染后续字段。

---

## 二、源码真实存在，但当前配置下休眠的问题

### B1. 手指控制复制粘贴 bug 真实存在，但当前配置不触发

**文件**: [task_driver.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/driver/task_driver.cpp#L339-L343), [driver.ini](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/config/driver.ini#L9-L22)

```cpp
jntCtrl.fingerJ[0]=dc.jntCtrlLoco.fingerJ[0];
jntCtrl.fingerJ[0]=dc.jntCtrlLoco.fingerJ[0];
```

当前配置：

```ini
fingerDofs=0,0
```

**复审结论**：源码 bug 成立，但当前 15 电机下肢配置没有手指自由度，因此此 bug 目前不影响当前配置运行。

如果将来切回带手指的全身配置，这个 bug 会重新变成活跃问题。

---

## 三、原报告有依据，但严重程度/技术表述需要收敛的问题

### C1. 共享内存三缓冲实现不够强健，但“必然混帧撕裂”的表述偏重

**文件**: [share_mem.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/src/common/share_mem.cpp#L118-L175)

写侧：

```cpp
int tmp=*pos;
tmp=(tmp+1)%3;
memcpy(memPtr+8+dataSize*tmp,data,dataSize);
(*pos)=tmp;
(*heartBeat)++;
```

读侧（vector 版）：

```cpp
int tmp=(*heartBeat);
char*head=memPtr+8+dataSize*(*pos);
if(heartBeatOld !=tmp){
    For(datas.size()){
        memcpy(datas[i],head,sizes[i]);
        head+=sizes[i];
    }
}
```

**复审结论**：这套实现确实不够严谨，但原报告中两点说得过头：

1. **vector 版并不是“多段 memcpy 间再次读取 `pos` 导致更严重混帧”**。实际上 `head` 只在开始时按 `*pos` 计算一次，后续都是在同一槽位上线性推进；
2. `heartBeat` 与 `pos` 分开读取，确实不是最稳健的快照方式，但它更像“可能读到更新/旧一帧”的一致性弱化，而不是原文描述的那种非常直接的“拼接新旧帧”证明。

**真正的问题**更准确地说是：

- 没有 seqlock/双重校验；
- 没有 reader 占用标记；
- 对“读时槽位未被覆盖”的证明依赖运行时调度假设；
- 因而这是**不够强健的 lock-free 共享内存实现**，但原报告把它直接定性成“已经证明会造成电机跳变损伤”的力度过大。

因此，这条建议**降级为高风险设计缺陷/健壮性问题**，而不是把结论写得像已经在当前配置中必然触发。

---

### C4. interface 的线程安全问题真实存在，但不是原报告描述的全部那种形态

**文件**: [task_interface.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface.cpp#L231-L245), [task_interface.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface.cpp#L249-L356)

**复审结论**：有真实的数据竞争，但原报告中至少两点需要修正。

#### 成立的部分

- `udpRcv()` 在后台线程里改写 `joy`、`dc.cmd`、`naviFlag`、`cctvFlag`；
- `work()` / `update()` 在主循环线程中也会读取其中一部分；
- 这些访问之间没有锁，也没有原子封装，因此**进程内确实存在数据竞争**。

#### 原报告写重的部分

1. **`recvBuf` 并不是并发覆盖**：
   `udpRcvOcu(recvBuf)`、`udpRcvCctv(recvBuf)`、`udpRcvNavi(recvBuf)` 等是在同一个 `udpRcv()` 线程里顺序调用的，不是多个线程同时收同一个缓冲；
2. **locomotion 不是直接读取半更新 `dc.cmd`**：
   真正跨进程传给 locomotion 的是 `dc.shmCmd.set(&dc.cmd)` 之后的共享内存快照，而不是另一个线程直接拿 `dc.cmd` 结构逐字段读。

因此，这条应改写为：

> interface 进程内部存在未同步的共享状态访问，可能造成单周期不一致；但原报告把它扩大成了跨进程控制链路上的“半更新命令直达 locomotion”，这一步推论过头了。

---

### M1. `ocuFeedbackStruct` 固定 31 项是协议耦合，当前更现实的问题是“尾部垃圾数据”

**文件**: [task_interface_data.h](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface_data.h#L143-L169), [task_interface.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface.cpp#L138-L170), [driver.ini](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/config/driver.ini#L19-L22)

**复审结论**：原报告“理论上会越界”这个判断在抽象层面没错，但就**当前配置**而言，`idMap` 只落在 0..14，不构成当前越界。

当前真正能从代码直接支持的问题是：

- `ocuFeed` 没有整体清零；
- 只填充了当前 `drvNums` 对应映射项；
- 剩余槽位可能带着旧值/未定义值被通过 UDP 发出去。

所以应把重点从“当前已经数组越界”改成：

> 这是一个固定 31 项协议结构，与当前 15 电机配置不匹配；虽然当前 `idMap` 不越界，但未使用槽位的数据完整性没有保证。

---

### M2. `sleep_until` 追赶执行确实存在，但更准确的问题是“无超限检测”

**文件**: [task_base.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/src/task_base/task_base.cpp#L61-L64)

```cpp
auto refClock=chrono::high_resolution_clock().now();
while(task.step()){
    refClock+=chrono::microseconds(us);
    this_thread::sleep_until(refClock);
}
```

**复审结论**：原报告描述的“落后后连续追赶”现象是存在的，但更准确的定性应该是：

- 这是一个**无 deadline overrun 检测**的固定周期循环；
- 一旦 `step()` 超时，会立即追赶；
- 这会让系统丧失对“已经错过多少周期”的可观测性。

但仅靠这段代码，不能直接断言“这就是严重 correctness bug”。它更像一个**实时性监测缺失**与调度策略问题。

---

### M3. `ftok("/home", mark)` 是脆弱实现，但原报告把“跨设备不可复现”讲重了

**文件**: [share_mem.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/src/common/share_mem.cpp#L60-L61)

```cpp
key_t key=ftok("/home",mark);
shmid=shmget(key,shmSize, 0666 | IPC_CREAT);
```

**复审结论**：这里确实有运维层面的脆弱性：

- 依赖固定路径；
- 系统异常退出时可能遗留 SysV SHM 段；
- 不是很便携。

但原报告里“不同机器上 `/home` inode 不同，因此不可复现”并不是这个工程的核心问题，因为这里本来就是**同机多进程 IPC**。

更准确的写法应是：

> 这是一个不够鲁棒的 key 生成方式，会给部署与残留清理带来维护负担，但不宜表述成主要架构性缺陷。

---

### M4 / M5. 相同优先级与同核绑定是调度风险，但不能直接写成“必然 EtherCAT 超时”

**文件**: [thread.ini](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/config/thread.ini#L1-L28), [task_driver.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/driver/task_driver.cpp#L159-L187)

`thread.ini`：

```ini
[driverSdk]
priority=88
cpu=1

[driverTask]
priority=88
cpu=2

[locoTask]
priority=88
cpu=2
```

driver 中：

```cpp
Ini::iniClass iniThread("../config/thread.ini");
sdkCpuId=iniThread.getVal("driverSdk", "cpu");
...
sdk.setCPU(sdkCpuId);
```

**复审结论**：

- `driverTask` 与 `locoTask` 同优先级、同 CPU 2 —— 这是真实的调度抖动风险；
- 但 EtherCAT 通信线程并不是简单地跟 `driverTask` 混在一起，它通过 `driverSdk.cpu` 单独指定了 CPU；
- 因此，从当前代码更稳妥的结论是：

> 这套线程配置会增加 driver/loco 控制回路的调度竞争与抖动风险，但不能仅凭这里就直接证明“driverTask 会把 EtherCAT 通信饿死”。

---

## 四、轻微但成立的问题

### D1. 死代码字符串字面量

**文件**: [task_driver.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/driver/task_driver.cpp#L321-L323)

```cpp
if(locoState!=Online){"loco离线";}
if(maniState!=Online){"mani离线";}
```

这是合法但无效的语句，说明原意大概率是想打印日志。问题成立，但影响很轻。

---

### D2. `pthread_attr_destroy` 缺失

**文件**: [task_base.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/src/task_base/task_base.cpp#L70-L82)

问题成立，但属于一次性资源清理不完整，不是主路径稳定性问题。

---

### D3. 未使用常量 `CntBase`

**文件**: [task_driver.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/driver/task_driver.cpp#L29)

```cpp
static const float CntBase=2000;
```

文件中后续直接写了大量 `cnt%2000`，并未使用该常量。这条成立，但只是整洁性问题。

---

## 五、建议如何拿这份报告去继续咨询

如果你打算把这份复审结果再拿去问别人，我建议把问题分成三类，而不是把所有条目都放在同一严重度里：

### 第一优先级：当前就值得追问或修复
1. `task_driver.cpp` 掉线后继续下发旧控制命令；
2. `task_interface.cpp` 析构/后台线程竞态；
3. `task_interface.cpp` 的 `posTarget[2]` / `posPerson[2]` 越界写。

### 第二优先级：确有风险，但需更精确描述
1. 共享内存三缓冲缺少更稳健快照验证；
2. interface 进程内数据竞争；
3. driver/loco 调度配置偏激进。

### 第三优先级：代码质量/运维层面问题
1. `ftok("/home", mark)`；
2. `pthread_attr_destroy` 缺失；
3. 死代码与未使用常量。

---

## 最终评价

原 `loong_base_audit.md` 的核心方向并没有完全跑偏：它确实抓到了几条真实问题。  
但如果目标是“拿给别人审、让别人相信”，那么原文最大的不足不是“发现了不存在的问题”，而是：

- **把部分风险写得过满**；
- **没有把当前配置是否触发讲清楚**；
- **没有区分‘真实 bug’和‘高风险设计缺陷’**；
- **还漏掉了一个可以直接落地复现的越界写问题**。

因此，更可信的结论应是：

> `loong_base-main` 当前存在若干真实缺陷，其中以“掉线后仍持续发送旧控制命令”和“interface 析构线程竞态”为优先级最高；其余部分条目应降级或改写为更精确的风险描述。
