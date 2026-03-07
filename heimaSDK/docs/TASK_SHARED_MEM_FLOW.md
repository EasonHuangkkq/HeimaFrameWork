# Task / SDK / Shared-Mem Dataflow (Driver, Interface, Locomotion)

This document explains the full logic chain, timeline, and nesting for:
- DriverSDK (ECAT/CAN/IMU)
- task_driver / task_interface / task_locomotion
- DataCenter + shared memory (share_mem)

All file paths below are from this repo.

## 1) Big Picture (data path)

```
ECAT/CAN/IMU
   |
   v
DriverSDK (drivers[] bound to ECAT domain buffers)
   |
   v
task_driver (reads SDK -> writes shared mem)
   |
   v
DataCenter (shared mem)
   |
   +--> task_locomotion (reads -> computes -> writes targets)
   +--> task_interface  (reads -> UDP feedback, writes cmd/app/jntSdk)
```

## 2) Nesting / ownership map

Driver process:
```
main/driver/main.cpp
  -> Task::driverTaskClass (periodicTaskClass)
     -> driverTaskClass::impClass
        -> Data::dataCenterClass singleton
           -> Mem::shareMemClass (one per data block)
        -> DriverSDK::DriverSDK singleton
           -> ECAT masters + rxtx threads
           -> drivers[] array (WrapperPair)
```

Locomotion process:
```
main/locomotion/main.cpp
  -> Task::locoTaskClass (periodicTaskClass)
     -> locoTaskClass::impClass
        -> Data::dataCenterClass singleton (attaches to same shm)
        -> Nabo locomotion core
```

Interface process:
```
main/interface/main.cpp
  -> Task::interfaceTaskClass (periodicTaskClass)
     -> interfaceTaskClass::impClass
        -> Data::dataCenterClass singleton (attaches to same shm)
        -> UDP receive thread
```

## 3) Shared memory design (share_mem)

Implementation: `heimaSDK/common/share_mem.h`, `heimaSDK/common/share_mem.cpp`

Memory layout per block:
```
[pos(int)][heartbeat(int)][data0][data1][data2]
```

Behavior:
- `set()` writes the next data block (pos = (pos+1) % 3) and increments heartbeat.
- `get()` copies the latest block only if heartbeat changed (unless enforce==true).
- Return value is a negative counter for offline detection (used by tasks).

Constraints:
- Only contiguous data can be passed directly.
- For non-contiguous data, use the vector<ptr,size> APIs.

## 4) DataCenter (shared-mem registry)

Definition: `heimaSDK/task_base/data_center.h`
Implementation: `heimaSDK/task_base/data_center.cpp`

DataCenter creates all shared memory blocks once using a fixed mark:
```
int mark = 2178;
shm.creat(mark, size);
mark++;
```

All processes construct DataCenter with the same order, so they attach to the
same shared memory segments (via ftok("/home", mark)).

Important data blocks:
- `jntSensLoco` (joint sensors, non-contiguous, shmBaseDataStruct)
- `jntCtrlLoco` (joint commands, non-contiguous, shmBaseDataStruct)
- `locoJntSdk` (external jnt-sdk input, non-contiguous, shmBaseDataStruct)
- `fingerSens` (finger joints, non-contiguous)
- `shmImu`, `shmCmd`, `shmAppIn`, `shmAppOut`, `shmLocoInfo`, `shmLocoOpenFlag`

## 5) SDK init and binding

Driver SDK init happens only in task_driver:
- `task_driver.cpp` calls `sdk.init(xml)`.
- `DriverSDK::impClass::init()` parses config and allocates `drivers[]`.
- Each `drivers[i].rx/tx` is bound to ECAT domain swap buffers.

So `drivers[]` live inside the SDK, not inside task_driver.

## 6) Runtime timeline (per cycle)

### 6.1 Driver task cycle (task_driver.cpp)

Order of calls inside `driverTaskClass::step()`:
1) `update()`
2) `work()`
3) `dwdate()`  (write commands to SDK)
4) `log()`

Key data flow:
- `sdk.getMotorActual(motActs)` -> fill `dc.jntSensLoco` -> `setToShm()`
- `sdk.getIMU(sdkImu)` -> fill `dc.imu` -> `shmImu.set()`
- `sdk.getDigitActual(fingerActs)` -> fill `dc.fingerSens` -> `setToShm()`

Then commands back to SDK:
- `dc.jntCtrlLoco.getFromShm()` -> build `motTgts`
- `sdk.setMotorTarget(motTgts)`

### 6.2 Locomotion task cycle (task_locomotion.cpp)

Order in `locoTaskClass::step()`:
1) `update()` -> read shared mem
2) `work()` -> Nabo step
3) `dwdate()` -> write shared mem

Reads:
- `dc.shmCmd.get(&dc.cmd)`
- `dc.shmImu.get(&dc.imu)`
- `dc.jntSensLoco.getFromShm()`
- `dc.fingerSens.getFromShm()`
- `dc.locoJntSdk.getFromShm()` (if updated)
- `dc.shmAppIn.get(&dc.appIn)`

Writes:
- `dc.jntCtrlLoco.setToShm()` (joint targets)
- `dc.shmLocoInfo.set(&dc.locoInfo)`
- `dc.shmLocoOpenFlag.set(&dc.locoOpenFlag)` (init / shutdown)

### 6.3 Interface task cycle (task_interface.cpp)

Order in `interfaceTaskClass::step()`:
1) `update()` -> read shared mem for feedback
2) `work()`   -> send UDP feedback
3) `dwdate()` (unused)

UDP receive thread:
- OCU/Navi -> `dc.shmCmd.set(&dc.cmd)`
- JntSdk -> `dc.locoJntSdk.setToShmDirectly(buf)`
- App -> `dc.shmAppIn.set(&dc.appIn)`

## 7) Data flow table (writer -> shared mem -> reader)

```
jntSensLoco  : driver  -> shm -> locomotion, interface
fingerSens   : driver  -> shm -> locomotion, interface
imu          : driver  -> shm -> locomotion, interface
jntCtrlLoco  : locomotion -> shm -> driver (for sdk.setMotorTarget)
locoJntSdk   : interface (UDP) -> shm -> locomotion
cmd          : interface (UDP) -> shm -> locomotion
appIn        : interface (UDP) -> shm -> locomotion (or other app)
appOut       : other app -> shm -> interface (UDP send)
locoInfo     : locomotion -> shm -> interface
locoOpenFlag : locomotion -> shm -> others
```

## 8) ID mapping (hardware order vs app order)

DataCenter loads `idMap` from `config/driver.ini`:
- `idMap[i]` maps app index -> hardware index.

task_driver uses this mapping:
- `motActPtrMaped[i] = &motActs[idMap[i]]`
- `motTgtPtrMaped[i] = &motTgts[idMap[i]]`

This means:
- Shared mem `jntSensLoco` and `jntCtrlLoco` are in **app order**.
- SDK `motActs` / `motTgts` are in **hardware order**.

## 9) ECAT rxtx thread (where SDK data comes from)

Per ECAT master, an rxtx thread:
- Copies Rx swap -> domain process image -> send
- Receives domain process image -> Tx swap -> app reads

So `sdk.getMotorActual()` reads from Tx swap (per domain buffer),
and `sdk.setMotorTarget()` writes to Rx swap.

Files:
- `heimaSDK/ecat.cpp` (rxtx)
- `heimaSDK/common.cpp` (SwapList copyTo/copyFrom)

## 10) Summary: end-to-end chain

```
ECAT/IMU -> DriverSDK -> task_driver -> DataCenter (shm)
DataCenter (shm) -> task_locomotion -> DataCenter (shm) -> task_driver -> DriverSDK -> ECAT
DataCenter (shm) <-> task_interface (UDP)
```

This is the full control loop with shared memory as the app-layer backbone.
