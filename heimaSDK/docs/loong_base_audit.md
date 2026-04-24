# loong_base-main 架构审计报告

> 审计范围：所有源码文件、配置文件、CMake 构建系统、预编译模块接口
> 审计视角：机器人框架安全性、实时性、正确性

---

## 🔴 严重问题（Critical）— 可能导致硬件损坏或人身伤害

### C1. 共享内存三缓冲存在 **数据撕裂（Data Tearing）**

**文件**: [share_mem.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/src/common/share_mem.cpp#L118-L137)

```cpp
// set() — 写入端
int tmp=*pos;
tmp=(tmp+1)%3;
memcpy(memPtr+8+dataSize*tmp, data, dataSize);
(*pos)=tmp;           // ① 先写数据，后更新 pos
(*heartBeat)++;

// get() — 读取端
int tmp=(*heartBeat);
if(heartBeatOld != tmp){
    memcpy(data, memPtr+8+dataSize*(*pos), dataSize);  // ② 读取时再次解引用 pos
}
```

**问题**：`set()` 和 `get()` 之间没有任何内存屏障或原子保护。虽然 `pos` 和 `heartBeat` 被声明为 `atomic<int>`，但关键数据块的 `memcpy` 本身不是原子的：

1. **写-读竞争**：`get()` 在 `memcpy` 执行过程中，`set()` 可能已经将 `pos` 更新为新值并开始写入新的块——但如果 `get()` 的 `memcpy` 尚未完成，而新的 `set()` 恰好写入了 `get()` 正在读取的同一个块（因为三缓冲循环 `(pos+1)%3` 可能回绕到 reader 正在读的块），就会发生撕裂
2. **`pos` 二次解引用**：`get()` 中读取 `heartBeat` 和读取 `*pos` 是两个独立的原子操作，中间 `set()` 可能已经更新了 `pos`，导致 `heartBeat` 对应的数据和 `pos` 指向的数据不一致
3. **vector 版 `get()`**（L154-175）更严重：多段 `memcpy` 之间如果 `pos` 被更新，则读取的是混合了新旧数据的拼接结果

**风险**：关节指令 `jntCtrlLoco` 中混合了两帧的 `j`（位置）和 `t`（力矩），可导致**电机瞬间跳变**，在真实机器人上可能**损坏减速器或造成摔倒**。

> [!CAUTION]
> 这个问题在 heimaSDK 的 `SwapList` 中你之前已经审计过（见历史对话），但 `loong_base-main` 的共享内存层使用了**完全不同的实现**，问题同样存在甚至更严重，因为这里是跨进程的。

**修复建议**：读端应在 `memcpy` 前后各读一次 `pos`，若两次不同则丢弃本次读取（seqlock 模式），或使用 `pos` 的快照值而非二次解引用：

```cpp
int get(void*data, bool enforce){
    int hb = heartBeat->load(memory_order_acquire);
    if(hb != heartBeatOld || enforce){
        int slot = pos->load(memory_order_acquire);  // 一次性快照
        memcpy(data, memPtr+8+dataSize*slot, dataSize);
        // 验证：memcpy 期间 pos 未被更新
        if(pos->load(memory_order_acquire) != slot){
            // 数据可能被撕裂，重新读取或标记
        }
        heartBeatOld = hb;
        getCnt = 1;
    }
    return --getCnt;
}
```

---

### C2. Driver 下发指令 **缺少看门狗保护（Watchdog）**

**文件**: [task_driver.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/driver/task_driver.cpp#L326-L419)

在 `dwdate()` 中，Driver 通过 `getFromShm()` 返回值判断 locomotion 是否在线：

```cpp
if(dc.jntCtrlLoco.getFromShm() > -100){  // 心跳计数 > -100 就认为在线
    locoState = Online;
    // ... 使用 jntCtrlLoco 的数据下发到电机
}
```

**问题**：

1. **阈值 `-100` 过于宽松**：如果 locomotion 进程崩溃，Driver 每 1ms 调用一次 `getFromShm()`，`getCnt` 每次 -1，需要 100ms 后才会检测到离线。在 100ms 内，Driver 会**持续下发最后一帧的旧指令**，如果此时机器人正在执行大幅运动，100ms 的旧指令会导致关节继续运动到危险位置
2. **离线后无安全动作**：当 `locoState==Lost` 被检测到后（L334-337），代码只打印了一行日志，**没有执行任何安全停机动作**。`dwdate()` 中后续的 `else` 分支（L381-395）是 `waitOP` 阶段的处理，不是离线保护
3. **无超时强制 disable**：如果 locomotion 长时间不上线，Driver 会一直在 `waitOP==0` 分支里用旧数据

**修复建议**：
```cpp
if(dc.jntCtrlLoco.getFromShm() > -5){  // 5ms 超时
    locoState = Online;
    // ... 正常下发
}else{
    if(locoState == Online){
        print("==loco离线！紧急制动==");
        locoState = Lost;
    }
    // 强制安全动作：保持当前位置，零速零力矩
    For(dc.drvNums){
        motTgts[i].pos = motActs[i].pos;
        motTgts[i].vel = 0;
        motTgts[i].tor = 0;
        motTgts[i].enabled = 0;  // 或 softDisable
    }
}
```

---

### C3. 右手手指指令 **复制粘贴 Bug**

**文件**: [task_driver.cpp L342-343](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/driver/task_driver.cpp#L342-L343)

```cpp
jntCtrl.fingerJ[0] = dc.jntCtrlLoco.fingerJ[0];
jntCtrl.fingerJ[0] = dc.jntCtrlLoco.fingerJ[0];  // ← 应为 fingerJ[1]
```

**两行完全一样**，右手手指指令 `fingerJ[1]` 永远不会从共享内存读取，保持初始化时的零值。如果机器人有灵巧手，右手将完全不响应控制指令。

---

### C4. Interface UDP 接收线程的 **线程安全问题**

**文件**: [task_interface.cpp](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface.cpp#L120-L245)

```cpp
void init(float dt){
    // ...
    thread udpTh(&interfaceTaskClass::impClass::udpRcv, this);
    udpTh.detach();  // 分离线程
}
```

`udpRcv()` 在独立线程中运行，但与主循环的 `step()` 共享以下数据且**无任何同步**：
- `dc.cmd`（L274-344 写 / L65-71 locomotion 读）
- `dc.appIn`（L448-450 写 / L113-116 locomotion 读）
- `joy` 结构体（L256-262 写 / L187-190 读）
- `cctvFlag`, `naviFlag`（L279-294 写 / L174 读）
- `recvBuf`（1MB 缓冲区，L237-241 所有 UDP 接收共用同一个 buffer）

**问题**：
1. `dc.cmd` 在 `udpRcvOcu()` 中被字段逐个赋值（L296, L310-318, L325-328），locomotion 进程可能在中间读到半更新的命令
2. 所有 UDP 接收函数共用同一个 `recvBuf`，如果某个 `recv()` 返回的数据在下一个 `udpRcvXxx()` 中被覆盖，但前一个还在用 cast 后的指针（如 L253 的 `ocuCmdStruct &tmp=*(ocuCmdStruct*)buf`），则会读到错误数据。实际上因为是顺序调用的所以不会并发覆盖，但设计意图不明确

---

## 🟠 重要问题（Major）— 影响可靠性和可维护性

### M1. `ocuFeedbackStruct` 硬编码 31 个关节，与 `drvNums` 不匹配

**文件**: [task_interface_data.h L143-169](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface_data.h#L143-L169)

```cpp
struct ocuFeedbackStruct{
    float posExp[31];   // 硬编码 31
    float posAct[31];
    // ... 全部 31
};
```

而 `driver.ini` 配置 `drvNums=15`。在 [task_interface.cpp L139-149](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface.cpp#L139-L149) 中：

```cpp
For(dc.drvNums){
    ocuFeed.posExp[dc.idMap[i]] = dc.jntCtrlLoco.j[i];
```

`dc.idMap[i]` 的值如果 ≥ 31（理论上对于全身 31 关节的配置不会，但如果配置出错则直接**数组越界**），没有边界检查。且当 `drvNums < 31` 时，数组中未填充的位置为**未初始化数据**，会通过 UDP 发送出去。

---

### M2. 定时循环使用 `sleep_until` 无超时检测

**文件**: [task_base.cpp L61-64](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/src/task_base/task_base.cpp#L61-L64)

```cpp
auto refClock = chrono::high_resolution_clock().now();
while(task.step()){
    refClock += chrono::microseconds(us);
    this_thread::sleep_until(refClock);
}
```

**问题**：如果 `step()` 执行时间超过 `dt`（如 1ms），`refClock` 会累积落后于当前时间，`sleep_until` 立即返回，导致**连续追赶执行**多个 `step()`。对于 1kHz 的 driver 任务，一次 2ms 的延迟会立即触发 2 次 `step()` 背靠背执行，可能引起：
- 共享内存写入频率突变
- 日志打印条件 `cnt%2000` 连续触发
- 无法感知实时性超限

**修复建议**：加入超限检测和丢帧机制：
```cpp
auto now = chrono::high_resolution_clock().now();
if(refClock < now){
    int missed = (now - refClock) / chrono::microseconds(us);
    if(missed > 2) print("WARNING: missed", missed, "cycles");
    refClock = now;  // 重置，不追赶
}
```

---

### M3. 共享内存标识 `ftok("/home", mark)` 的脆弱性

**文件**: [share_mem.cpp L60](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/src/common/share_mem.cpp#L60)

```cpp
key_t key = ftok("/home", mark);
```

**问题**：
1. `ftok` 依赖文件的 inode 号，如果 `/home` 被重新挂载或 inode 变化（系统更新、磁盘迁移），所有共享内存的 key 都会改变，导致进程间无法通信
2. 不同机器上 `/home` 的 inode 不同，无法跨设备复现问题
3. 旧的共享内存段在进程异常退出后不会被清理（`IPC_RMID` 只在最后一个 detach 时标记），需要手动 `ipcrm`

---

### M4. 所有进程优先级相同（priority=88），无优先级分层

**文件**: [thread.ini](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/config/thread.ini)

```ini
[driverTask]  priority=88
[locoTask]    priority=88
[interfaceTask] priority=88
```

在实时系统中，所有任务使用相同的 `SCHED_FIFO` 优先级意味着**调度完全取决于到达顺序**。如果 interface 或 locomotion 的某次循环耗时过长，它可以**饿死 driver 进程**（如果在同一 CPU 上），导致 EtherCAT 通信超时。

**推荐优先级**：`driver (98) > locomotion (88) > interface (68)`

---

### M5. driverTask 和 locoTask 共享 CPU 核 2

**文件**: [thread.ini](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/config/thread.ini#L7-L17)

```ini
[driverTask]  cpu=2
[locoTask]    cpu=2  ← 同一个核！
```

结合 M4 的相同优先级问题，两个 1kHz 的实时任务跑在同一个核上，会产生严重的**调度抖动**。Driver 的 EtherCAT 通信有严格的时序要求（通常 < 100μs 抖动），与 locomotion 共享 CPU 无法保证。

---

## 🟡 一般问题（Minor）— 代码质量和可维护性

### m1. 死代码：状态字符串字面量未使用

**文件**: [task_driver.cpp L322](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/driver/task_driver.cpp#L322)

```cpp
if(locoState!=Online){"loco离线";}    // 字符串字面量，什么都不会做
if(maniState!=Online){"mani离线";}    // 应该是 print("loco离线")
```

---

### m2. `interfaceTaskClass` 析构函数的竞态

**文件**: [task_interface.cpp L463-466](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/interface/task_interface.cpp#L463-L466)

```cpp
~interfaceTaskClass(){
    imp.udpRunning = 0;   // ① 设置标志
    delete &imp;           // ② 立即 delete，不等待线程结束
}
```

`udpRcv()` 线程使用了一个**局部变量** `bool udpRunning=1`（L233）而非成员变量 `this->udpRunning`，所以 ① 的赋值完全无效。此外即使修正，② 在线程还在运行时就 `delete` 了 `imp`，会导致**悬空指针崩溃**。

---

### m3. `pthread_attr_destroy` 缺失

**文件**: [task_base.cpp L75-81](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/src/task_base/task_base.cpp#L75-L81)

```cpp
pthread_attr_init(&tattr);
// ... 设置属性 ...
pthread_create(&threadId, &tattr, loop, arg);
// 缺少 pthread_attr_destroy(&tattr);
```

虽然影响很小（少量内存泄漏），但在长期运行的机器人系统中应注意。

---

### m4. `CntBase=2000` 定义但未使用

**文件**: [task_driver.cpp L29](file:///Users/shenghuang/Downloads/heima/heimaFrameWork/loong_base-main/main/driver/task_driver.cpp#L29)

```cpp
static const float CntBase=2000;  // 从未被引用
```

---

## 📊 问题汇总

| 级别 | ID | 问题 | 影响 |
|------|-----|------|------|
| 🔴 Critical | C1 | 共享内存数据撕裂 | 关节指令跳变→硬件损坏 |
| 🔴 Critical | C2 | 缺少看门狗，离线无安全动作 | locomotion 崩溃后持续下发旧指令 |
| 🔴 Critical | C3 | 右手手指复制粘贴 Bug | 右手完全失控 |
| 🔴 Critical | C4 | UDP 线程与主循环无同步 | cmd 数据撕裂 |
| 🟠 Major | M1 | 反馈结构体硬编码 31 关节 | 数组越界/发送垃圾数据 |
| 🟠 Major | M2 | 无超时检测的定时循环 | 实时性丢失时连续追赶 |
| 🟠 Major | M3 | ftok 依赖 /home inode | 跨设备不可复现 |
| 🟠 Major | M4 | 所有任务同优先级 | driver 可被饿死 |
| 🟠 Major | M5 | driver 和 loco 共享 CPU | 调度抖动 |
| 🟡 Minor | m1 | 字符串字面量死代码 | 离线检测无效 |
| 🟡 Minor | m2 | 析构函数竞态+局部变量遮蔽 | 进程退出崩溃 |
| 🟡 Minor | m3 | pthread_attr 未销毁 | 内存泄漏 |
| 🟡 Minor | m4 | 未使用常量 | 代码整洁 |

---

## 架构层面的整体评价

### 设计优点
1. **进程隔离**：driver/locomotion/interface 独立进程，崩溃隔离思路正确
2. **pImpl 模式**：接口干净，编译依赖隔离
3. **DataCenter 单例**：统一的数据注册中心，清晰
4. **关节约束层**：driver 中对 pos/vel/kp/kd/tor 全部做了 clip，是重要的安全层

### 核心设计缺陷
1. **缺少心跳/看门狗的全链路设计**：只有 locomotion→driver 的"负计数"掉线检测，且阈值过松。应该有双向心跳 + 分级安全动作（软停→硬停→断电）
2. **共享内存的无锁设计不完整**：三缓冲的思路正确，但实现上缺少必要的内存屏障和读验证，在多核 ARM（如 Jetson Thor）上尤其危险
3. **没有进程启动顺序和依赖管理**：三个进程独立启动，如果 locomotion 先于 driver 启动，读到的传感器数据全是零，可能导致算法初始化异常
