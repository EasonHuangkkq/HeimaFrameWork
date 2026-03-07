# loong_base a64 Driver Bring-up (Heima SDK, 15 Motors)

## Context

### Original Request
Run through `loong_base-main` to adapt it to a Heima-modified DriverSDK (same API names as Loong), but **start by getting only the driver task running** on the a64 real robot.

### Interview Summary (Confirmed)
- Target runtime: **a64 real robot**.
- Integration approach: **keep DriverSDK API/ABI compatible**, but output library name can remain “heima”; use **symlinks/aliases** so `loong_base-main` still resolves expected names.
- Bring-up scope: **15 motors only**: left leg 6 + right leg 6 + lumbar 3.
- Bus / SDK enumeration order: **left leg (6) -> right leg (6) -> lumbar (3)**.
- SDK XML filename to use: `config.xml`.
- Defer: interface/OCU, locomotion, and any full-body 31/33-DOF support.

### Key Code Facts (Evidence)
- The **only process calling DriverSDK** is the driver task: `loong_base-main/main/driver/task_driver.cpp`.
- Driver hard-checks motor count: `dc.drvNums != sdk.getTotalMotorNr()` throws.
- `idMap` semantics: `pino[i] = hardware[idMap[i]]`:
  - `loong_base-main/src/task_base/data_center.h` (comment)
  - `loong_base-main/main/driver/task_driver.cpp` maps `motActPtrMaped[i]=&motActs[dc.idMap[i]]`
- Driver links fixed library names: `loong_driver_sdk_${tgtArch}` + `joint_map_${tgtArch}` via `loong_base-main/main/driver/CMakeLists.txt`.

### Metis Review (Gaps Addressed)
Metis highlighted:
- Safety posture must be explicit for real robot bring-up.
- `joint_map_a64` may assume full-body dimensions; verify it does not break 15-motor mode.
- Build/deploy toolchain ABI mismatch risk; validate with `ldd`/runtime loader.
- Config resolution depends on working directory (`../config`).

### Required Inputs (Bring-up Artifacts)
This plan assumes you have already-tested Heima artifacts available:
- `HEIMA_DRIVER_SDK_SO`:
  - aarch64 shared library implementing `namespace DriverSDK` exactly matching `loong_base-main/module/loong_driver_sdk/loong_driver_sdk.h`
- `HEIMA_CONFIG_XML_15`:
  - a 15-motor `config.xml` (L leg 6 -> R leg 6 -> lumbar 3)

---

## Work Objectives

### Core Objective
Start `loong_driver_a64` reliably on the robot with the Heima SDK, **reading feedback for 15 motors** and staying safe.

### Concrete Deliverables
- A runnable `loong_base-main` driver setup for “lower body 15 motors”.
- A `loong_base-main/config/` that satisfies `../config/*.ini` + `sdkXml` resolution.
- A Heima driver `.so` installed under `loong_base-main/module/loong_driver_sdk/` with compatibility symlink for `libloong_driver_sdk_a64.so`.

### Definition of Done
- `sudo ./loong_driver_a64` starts (from expected run location), resolves `../config/driver.ini` + `../config/config.xml`, and does not throw.
- `sdk.getTotalMotorNr()` equals `drvNums=15` and the driver passes the mismatch check.
- Periodic prints/logs show 15 motors’ feedback updating (pos/vel/tor/status/error/temp).

### Must NOT Have (Guardrails)
- Do NOT start interface/OCU or locomotion processes during this bring-up.
- Do NOT modify OCU protocol structs (they contain fixed `[31]` arrays in `loong_base-main/main/interface/task_interface_data.h`).
- Do NOT change DriverSDK public API/ABI.
- Do NOT run anything that can cause unexpected motion without a deliberate enable decision.

---

## Verification Strategy

### Test Decision
- Automated tests: NO (hardware bring-up)
- QA approach: Manual verification + log evidence

### Safety Mode (DECISION NEEDED)
Pick one safety posture for the very first runs:
- Option A (recommended): **feedback-first** (no enable/torque).
  - In driver-only bring-up, this is achieved by ensuring no other process writes `DataCenter.jntCtrlLoco` and that shared-memory is in a known state.
- Option B: allow enable but enforce “no motion” (targets hold current position, torque limited).

Until decided, default the run sequence to **Option A**.

Evidence required:
- Photos/video of robot safely supported (or legs off-ground), E-stop accessible.
- Terminal logs saved under `loong_base-main/log/terminal_driver.txt`.

---

## Task Flow

1) Prepare runtime directory layout and configs
2) Build Heima SDK `.so` for a64 and place it under module path
3) Provide compatibility symlinks for library names and config path
4) Build or use `loong_driver_a64`
5) Run driver in feedback-first mode and validate 15-motor feedback + stable EtherCAT state

---

## TODOs

### 1) Create a dedicated `loong_base-main/config/` for 15-motor bring-up

**What to do**:
- Create directory `loong_base-main/config/` (do not rely solely on `loong_sim_sdk_release-main/config/` unless you intentionally want shared edits).
- Populate minimal config set:
  - `loong_base-main/config/driver.ini`
  - `loong_base-main/config/thread.ini`
  - (optional) `loong_base-main/config/interface.ini` (not needed for driver-only)
- Copy `config.xml` into `loong_base-main/config/config.xml`.

**Minimal `thread.ini` requirement (do not skip)**:
Driver will throw if `../config/thread.ini` is missing required keys.

```ini
[driverSdk]
priority=88
cpu=1

[driverTask]
priority=88
cpu=2
dt=0.001
```

You can copy this from `loong_sim_sdk_release-main/config/thread.ini`.

**References**:
- `loong_base-main/src/task_base/data_center.cpp` uses `../config/driver.ini`.
- `loong_base-main/main/driver/task_driver.cpp` builds `sdkXml = "../config/" + iniDrv.getStr("sdkXml")`.
- `loong_base-main/tools/run_driver.sh` executes from `../bin` or `../nabo_output`, so `../config` must exist relative to those.

**Acceptance Criteria**:
- From the directory where you will run the executable (typically `loong_base-main/bin/`): `ls ../config` shows `driver.ini`, `thread.ini`, `config.xml`.
- `driver.ini` contains `sdkXml=config.xml`.

### 1.1) Ensure shared-memory is in a known safe state (avoid stale enable/targets)

**Why**:
Shared memory is only zeroed when the segment is first attached (`shm_nattch==1` in `loong_base-main/src/common/share_mem.cpp`). If old processes are still attached or leaked segments exist, you may inherit stale `enable/targets` after OP.

**What to do**:
- Before first bring-up run, ensure:
  - no `loong_*` processes are running,
  - you start the driver first,
  - ideally after a fresh reboot of the robot.
- If you cannot reboot, verify with `ipcs -m` that no unexpected SysV shm segments remain from prior runs, and remove only the relevant ones.

**Deterministic cleanup (SysV shm keys for this stack)**:
`DataCenter` starts at `mark=2178` and creates 10 shm segments sequentially (marks 2178..2187).
`share_mem` uses `ftok("/home", mark)`.

On the robot, compute the exact keys and remove them:

```bash
python3 - <<'PY'
import ctypes
libc = ctypes.CDLL('libc.so.6')
libc.ftok.argtypes = [ctypes.c_char_p, ctypes.c_int]
libc.ftok.restype = ctypes.c_int
for mark in range(2178, 2188):
    key = libc.ftok(b'/home', mark)
    print(mark, hex(ctypes.c_uint(key).value))
PY
```

Then for each printed key:

```bash
ipcrm -M 0x........
```

**Acceptance Criteria (observable)**:
- During driver-only bring-up, the driver must NOT print `==loco上线！==` (meaning no locomotion process is publishing shared-memory control).

### 2) Edit `driver.ini` for 15-motor lower-body topology

**What to do**:
- Set the topology block:
  - `drvNums=15`
  - `armDof=0`
  - `fingerDofs=0,0`
  - `neckDof=0`
  - `lumbarDof=3`
  - `legDof=6`
- Set `sdkXml=config.xml`.

**idMap for this bring-up**:
The driver internally uses pino grouping order: `l arm, r arm, neck+lumbar, l leg, r leg`.
With `armDof=0` and `neckDof=0`, pino order becomes: `lumbar(3) -> left leg(6) -> right leg(6)`.

Given SDK/bus order is confirmed as: `left leg(6) -> right leg(6) -> lumbar(3)` (indices 0..5, 6..11, 12..14), set:

`idMap = 12,13,14, 0,1,2,3,4,5, 6,7,8,9,10,11`

**Joint parameter arrays**:
- Ensure `[joint]` arrays have **exactly 15 values** each (`maxCur`, `motBiasCnt`, `kpMax`, `kdMax`, `maxPos`, `minPos`, `maxVel`, `maxTor`, `cutFrq`).
- Default (safe for feedback-first): use conservative values for first bring-up (especially `maxCur`/`maxTor`).

**Recommended starting template** (adjust later when you start commanding motion):

```ini
sdkXml=config.xml

drvNums=15
armDof=0
fingerDofs=0,0
neckDof=0
lumbarDof=3
legDof=6

imuAdj=0,0
logCnt=10

; pino order is: lumbar(3) -> left leg(6) -> right leg(6)
; bus order is:  left leg(6) -> right leg(6) -> lumbar(3)
idMap=12,13,14, 0,1,2,3,4,5, 6,7,8,9,10,11

[joint]
maxCurRate=1

; Conservative defaults for feedback-first.
; Keep maxTor=0 and kp/kd=0 to ensure commanded torque stays 0.
maxCur=500,500,500, 500,500,500,500,500,500, 500,500,500,500,500,500
motBiasCnt=0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0
kpMax=0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0
kdMax=0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0
maxPos=2,2,2, 2,2,2,2,2,2, 2,2,2,2,2,2
minPos=-2,-2,-2, -2,-2,-2,-2,-2,-2, -2,-2,-2,-2,-2,-2
maxVel=10,10,10, 10,10,10,10,10,10, 10,10,10,10,10,10
maxTor=0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0
cutFrq=500,500,500, 500,500,500,500,500,500, 500,500,500,500,500,500
```

**References**:
- `loong_base-main/src/task_base/data_center.cpp` reads `drvNums/armDof/fingerDofs/neckDof/lumbarDof/legDof/idMap`.
- `loong_base-main/main/driver/task_driver.cpp` loads `[joint]` arrays by `dc.drvNums`.

**Acceptance Criteria**:
- `driver.ini` parses without errors.
- `idMap` has 15 integers.
- Each `[joint]` array line count/values match 15.

### 3) Build Heima DriverSDK shared library for a64 and stage it for loong_base

**What to do**:
- Build your Heima driver SDK for a64 and produce a shared library (example name: `libheima_driver_sdk_a64.so` or `libheima_driver_sdk.so`).
- Ensure the library exports the expected C++ symbols under `namespace DriverSDK` and that `getTotalMotorNr()` returns 15 for your `config.xml`.
- Ensure the resulting `.so` is compatible with the a64 robot’s runtime (glibc/libstdc++ versions).
- Copy it into:
  - `loong_base-main/module/loong_driver_sdk/`

**Source-of-truth requirement (no guessing)**:
- Use YOUR Heima SDK source repo that you already validated on hardware.
- The produced `.so` MUST implement `namespace DriverSDK` matching exactly what `loong_base-main` compiles against:
  - header: `loong_base-main/module/loong_driver_sdk/loong_driver_sdk.h`

**ABI guardrail**:
- Do not mix headers: the consumer header must be `loong_driver_sdk.h` from `loong_base-main`.
- Verify the staged `.so` exports the expected symbols:
  - `DriverSDK::DriverSDK::instance()`
  - `DriverSDK::DriverSDK::init(char const*)`
  - `DriverSDK::DriverSDK::getTotalMotorNr()`
  - `DriverSDK::DriverSDK::getMotorActual(...)`
  - `DriverSDK::DriverSDK::setMotorTarget(...)`

Suggested checks (run on a64):
- `nm -D <your_so> | c++filt | grep 'DriverSDK::DriverSDK::instance'`
- `ldd <your_so>`

**Compatibility symlink (required unless you change loong_base CMake)**:
- Create symlink in the same directory:
  - `libloong_driver_sdk_a64.so -> <your heima sdk .so>`

**References**:
- Link name expectations: `loong_base-main/main/driver/CMakeLists.txt` links `loong_driver_sdk_${tgtArch}`.
- SDK API header used by driver: `loong_base-main/module/loong_driver_sdk/loong_driver_sdk.h`.

**Acceptance Criteria**:
- `ls -l loong_base-main/module/loong_driver_sdk/` shows both the heima `.so` and `libloong_driver_sdk_a64.so` symlink.
- On the robot: `ldd loong_base-main/module/loong_driver_sdk/libloong_driver_sdk_a64.so` reports no missing deps.

### 3.1) SDK probe (objective verification of ABI + XML motor count)

**Goal**:
Verify the staged `.so` is linkable with the *consumer* header (`loong_driver_sdk.h`) and that `config.xml` yields `getTotalMotorNr()==15`.

**What to do**:
1) Create a tiny probe source file (example path): `loong_base-main/module/driver_sdk_probe.cpp`:

```cpp
#include "loong_driver_sdk/loong_driver_sdk.h"
#include <iostream>

int main(){
  auto &sdk = DriverSDK::DriverSDK::instance();
  sdk.init("../config/config.xml");
  std::cout << "totalMotors=" << sdk.getTotalMotorNr() << std::endl;
  return 0;
}
```

2) Build and run it on the a64 robot, using the same header and library search path as the driver:

```bash
cd loong_base-main/module
g++ -std=c++17 -O2 -I. -I../src -I../../loong_utility-main/src \
  -L./loong_driver_sdk -Wl,-rpath,'$ORIGIN/loong_driver_sdk' \
  driver_sdk_probe.cpp -lloong_driver_sdk_a64 -o driver_sdk_probe

./driver_sdk_probe
```

**Acceptance Criteria**:
- Output contains `totalMotors=15`.
- No runtime loader errors (missing libs) when running `./driver_sdk_probe`.

### 3.2) Clarify source of `config.xml` and verify it matches the 15-motor topology

**What to do**:
- Preferred: use your Heima SDK repo’s source-of-truth 15-motor `config.xml`.
- Workspace note: `heimaSDK_copy/config.xml` is a full-body config and is NOT a valid fallback for this 15-motor bring-up.
- Confirm it enumerates exactly 15 motors in bus order: L leg 6 -> R leg 6 -> lumbar 3.

If you only have a full-body XML right now:
- Create a dedicated `config_15.xml` by removing all non-lower-body motors, then rename/copy to `loong_base-main/config/config.xml`.
- Use the SDK probe (Task 3.1) to confirm it reports 15.

**Acceptance Criteria (objective)**:
- `loong_driver_a64` passes the hard check (no “驱动数目不匹配！”), which implies `sdk.getTotalMotorNr()==15`.

### 4) Validate `joint_map_a64` behavior for 15-motor mode

**Why this matters**:
Driver calls `JntMap::update`/`JntMap::dwdate` unconditionally when not `DefSim`.
If `libjoint_map_a64.so` assumes full-body dimensions, it could crash or corrupt data when vectors are size 15.

**What to do**:
- Confirm `libjoint_map_a64.so` can accept `vecXf` sizes of 15 (or provide a compatible replacement).

**References**:
- Calls:
  - `loong_base-main/main/driver/task_driver.cpp` calls `JntMap::update` and `JntMap::dwdate`.
- Interface:
  - `loong_base-main/module/loong_driver_sdk/joint_map.h`.

**Acceptance Criteria**:
- Driver runs past the first control loop iterations after OP without crashing.
- No obvious NaNs/garbage in printed “下发 mot pos” / “上传 jnt pos”.

**Fallback (if it fails)**:
- Provide an identity-map implementation of `libjoint_map_a64.so` (no-op `update()`/`dwdate()`) for the 15-motor bring-up.
- Keep the exported symbol names exactly as declared in `loong_base-main/module/loong_driver_sdk/joint_map.h`.

**Executable fallback procedure (drop-in)**:
1) Create `loong_base-main/module/loong_driver_sdk/joint_map_identity.cpp`:

```cpp
#include "eigen.h"

namespace JntMap {
extern "C" void update(vecXf&, vecXf&, vecXf&) {}
extern "C" void dwdate(vecXf&, vecXf&, vecXf&) {}
}
```

2) Build on the a64 robot (adjust include paths to your checkout location):

```bash
cd loong_base-main/module/loong_driver_sdk
g++ -shared -fPIC -O2 joint_map_identity.cpp -o libjoint_map_a64.so \
  -I../../../loong_utility-main/src \
  -I../../src \
  -I..
```

3) Backup and replace the original `libjoint_map_a64.so` in the same directory.

### 5) Build and run `loong_driver_a64` (driver-only bring-up)

**What to do**:
- Choose ONE build workflow and document it in the execution notes:
  - Workflow A (cross-build on x64 host): `./tools/make.sh ar` (requires `/usr/bin/aarch64-linux-gnu-g++-9`) then copy `loong_base-main/nabo_output/loong_driver_a64` to the robot.
  - Workflow B (use existing a64 binary): use a known-good `loong_driver_a64` binary (e.g., from your release/robot image) and only swap in the module `.so` + config.

**Run location gotcha (script behavior)**:
- `loong_base-main/tools/make.sh` installs executables to `loong_base-main/nabo_output/` (`CMAKE_INSTALL_PREFIX ../nabo_output` in `loong_base-main/CMakeLists.txt`).
- `loong_base-main/tools/run_driver.sh` runs from:
  - `../bin` when called with no args,
  - `../nabo_output` when called with any arg.

For bring-up, easiest is:
- Run from `loong_base-main/tools/` and call `./run_driver.sh s` so it executes `../nabo_output/loong_driver_a64`.
- Alternatively, create `loong_base-main/bin -> loong_base-main/nabo_output` symlink to keep `./run_driver.sh` (no args) working.

**Run location requirement**:
- Ensure the executable’s working directory matches the scripts’ expectation so `../config/*` resolves.

**References**:
- Build script location requirement: `loong_base-main/readme.md`.
- Run script: `loong_base-main/tools/run_driver.sh`.

**Acceptance Criteria**:
- Terminal output includes periodic sections such as:
  - `>>上传 mot pos ...` / `>>状态字 ...` / `>>上传 jnt pos ...`
- No exception: “驱动数目不匹配！”
- `OP驱动数=15` appears at least once.
- `loong_base-main/log/terminal_driver.txt` exists and captures the session.

**Operational warning**:
- `loong_base-main/tools/run_driver.sh` deletes logs beyond the newest 20 entries in `loong_base-main/log/` using `sudo rm -f`. If you need to preserve logs, copy them out before reruns.

### 6) (Optional) EtherCAT health verification

**What to do**:
- Use EtherCAT tooling (if available) to verify slaves are discovered and state is stable.

**Acceptance Criteria**:
- `ethercat slaves` output shows the expected 15 devices and stable state during driver run.

---

## Commit Strategy
No commits required for bring-up planning unless you decide to persist configuration changes in-repo.

---

## Success Criteria

### Verification Commands (examples)
- Build: `cd loong_base-main/tools && ./make.sh ar`
- Run (from nabo_output): `cd loong_base-main/tools && ./run_driver.sh s`
- Logs: `ls loong_base-main/log/terminal_driver.txt`

### Final Checklist
- [ ] `drvNums=15` matches `sdk.getTotalMotorNr()`.
- [ ] `idMap` matches (lumbar -> L leg -> R leg) pino order.
- [ ] Driver runs stable in feedback-first mode.
- [ ] No interface/locomotion started; no OCU dependency.

---

## Runtime Dependencies (a64)

`loong_base-main` sets rpath to find third-party libs relative to the executable (see `loong_base-main/CMakeLists.txt`).

**Expected location**:
- `loong_base-main/third_party/lib_lin_a64/` should contain (at least):
  - `libethercat.so`
  - `libmodbus.so`
  - `libtinyxml2.so`
  - (optional) `libonnxruntime.so*` if pulled indirectly

**Acceptance Criteria**:
- `ldd loong_base-main/nabo_output/loong_driver_a64` shows all dependencies resolved.
- If `ldd` reports a missing library, place it into `loong_base-main/third_party/lib_lin_a64/` (preferred, matches rpath) or set `LD_LIBRARY_PATH` explicitly for bring-up.
