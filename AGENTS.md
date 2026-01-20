# AGENTS.md — heimaFrameWork workspace instructions

This repository is a *workspace bundle* of several OpenLoong-related subprojects (flattened as `*-main/`) plus a driver SDK source copy (`heimaSDK_copy/`). Each subproject has its own build/run scripts and slightly different conventions.

## Scope + existing agent rules
- Root-level agent instructions live in this file.
- Additional agent rules exist at:
  - `heimaSDK_copy/AGENTS.md` (applies only inside `heimaSDK_copy/`).
- No Cursor rules were found (`.cursorrules` or `.cursor/rules/`).
- No Copilot rules were found (`.github/copilot-instructions.md`).

## Repo map (high level)
- `heimaSDK_copy/`: EtherCAT/CAN/RS232/RS485 DriverSDK source (hardware-facing).
- `loong_utility-main/`: shared C++ utility library + style doc (`代码规范new.md`).
- `loong_ctrl_locomotion-main/`: locomotion algorithm library (Nabo-style, produces `libnabo_*.so`).
- `loong_base-main/`: multi-process “business main” (driver/locomotion/interface tasks).
- `loong_sim-main/`: MuJoCo algorithm simulation (produces `main_loco`).
- `loong_sim_sdk_release-main/`: prebuilt “full-chain simulation SDK” (binaries + configs + python examples).

## Build / lint / test (by subproject)

### Important: run scripts from their `tools/` directory
Many scripts rely on `cd ..` and do **not** resolve paths relative to the script location.

### Important: workspace symlinks / expected folder names
Some CMake files assume the original repo layout (e.g. `include(../loong_utility/loong_utility.cmake)`), but this workspace uses `loong_utility-main/`.
- If you see CMake errors like “include could not find requested file”, create a symlink at the workspace root:
  - `ln -s loong_utility-main loong_utility`
- Some helper scripts also reference `loong_sim/` or `loong_base/` (without `-main`). Prefer fixing via symlink rather than editing build scripts unless explicitly requested.

#### `heimaSDK_copy/` (DriverSDK)
- CMake build (library + example):
  - `cd heimaSDK_copy`
  - `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`
  - `cmake --build build --target driver_sdk`
  - `cmake --build build --target run_motor`
- “robot2 EtherCAT interface” shared lib:
  - `cd heimaSDK_copy`
  - `./build.sh` (or cross-compile: `CC=aarch64-linux-gnu-gcc ./build.sh`)
- Single target build:
  - `cmake --build build --target driver_sdk`

#### `loong_ctrl_locomotion-main/` (locomotion library)
- Scripted build (recommended by repo docs):
  - `cd loong_ctrl_locomotion-main/tools`
  - `./make.sh xs`  # x64 sim
  - `./make.sh xr`  # x64 robot (or co-sim)
  - `./make.sh as`  # aarch64 sim
  - `./make.sh ar`  # aarch64 robot
- What it does (simplified): creates build dir → runs CMake → `make -j6 install`.
- Manual build (equivalent to `xs`):
  - `cd loong_ctrl_locomotion-main`
  - `mkdir -p build_x64_sim && cd build_x64_sim`
  - `cmake .. -Dcpu_x64=1 -Dsim_mode=1 -Drl_use_cuda=0`
  - `make -j6 install`
- Single target build:
  - `cmake --build build_x64_sim --target nabo_x64`
  - `cmake --build build_x64_sim --target main` (example executable)

#### `loong_base-main/` (task executables)
- Scripted build:
  - `cd loong_base-main/tools`
  - `./make.sh xs|xr|as|ar`
- Manual build (equivalent to `xs`):
  - `cd loong_base-main`
  - `mkdir -p build_x64_sim && cd build_x64_sim`
  - `cmake .. -Dcpu_x64=1 -Dsim_mode=1`
  - `make -j6 install`
- Single target build (examples):
  - `cmake --build build_x64_sim --target loong_driver_x64`
  - `cmake --build build_x64_sim --target loong_locomotion_x64`
  - `cmake --build build_x64_sim --target loong_interface_x64`
  - `cmake --build build_x64_sim --target test` (from `loong_base-main/main/test`)

#### `loong_sim-main/` (MuJoCo simulation)
- Build:
  - `cd loong_sim-main/tools && ./make.sh`
  - (script does: `mkdir -p ../build && cd ../build && cmake .. && make -j4 install`)
- Run:
  - `cd loong_sim-main/tools && ./run_mujoco_loco.sh [delay_ms]`
- Single target build:
  - `cmake --build loong_sim-main/build --target main_loco`

#### `loong_sim_sdk_release-main/` (prebuilt runtime package)
- This directory is primarily *run-time artifacts*.
- Run scripts exist in `loong_sim_sdk_release-main/tools/` (e.g. `run_sim.sh`, `run_driver.sh`, `run_interface.sh`).

### Lint / formatting
- No repository-wide linter/formatter config was found (no `.clang-format`, `.editorconfig`, ESLint/Prettier, etc.).
- Prefer **local, minimal formatting** matching surrounding code; do not mass-reformat.

### Tests
- There is no standardized unit-test runner (no `ctest` suite configured).
- “Tests” are typically ad-hoc executables or hardware/sim smoke runs.
- For a single “test-like” target:
  - `loong_base-main`:
    - build: `cd loong_base-main/build_x64_sim && cmake --build . --target test`
    - run: `cd loong_base-main/build_x64_sim && ./main/test/test`
  - `loong_ctrl_locomotion-main`:
    - build: `cd loong_ctrl_locomotion-main/build_x64_sim && cmake --build . --target main`
    - run: `cd loong_ctrl_locomotion-main/build_x64_sim && ./main`

## Developer tooling
### clangd / compile_commands.json
- `clangd` is available in this environment.
- For better IDE/LSP support, configure CMake builds with:
  - `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

## Code style + conventions (C/C++)

### Canonical style doc
- Follow `loong_utility-main/代码规范new.md` for the `loong_*` projects unless the local directory’s code indicates otherwise.

### Formatting (observed in `loong_*`)
- Compact style is common:
  - `if(x){...}else{...}` (braces on same line)
  - Often no space after keywords: `if(`, `for(`, `while(`
  - Includes commonly appear as `#include"file.h"` or `#include<vec>` (no space after `#include`).
- Many files use tabs for indentation; preserve the existing file’s indentation style.

### Naming
- Files: lower_snake_case (see `share_mem.*`, `data_center.*`).
- Variables (compute-heavy code): lowerCamelCase, avoid underscores.
- Types:
  - `struct` = data container (may have `init/reset`), minimal logic.
  - `class` = encapsulates logic.
  - Common suffix pattern in code: `*Class` / `*Struct` (e.g. `shareMemClass`, `logClass`).
- Units & frames (from `代码规范new.md`):
  - Default units are SI (meters, radians, seconds, kilograms, Newton, N·m).
  - Coordinate frame: z-up, x-forward, right-handed.
  - Euler order: zyx (yaw/pitch/roll).

### Types / math
- Prefer project aliases from `loong_utility-main/src/eigen.h`:
  - `vec3f`, `vecXf`, `mat3d`, etc.
- Avoid hidden allocations in high-frequency loops.

### Includes and dependencies
- Avoid pulling uncommon/heavy dependencies into headers.
- pImpl is used in multiple places (pattern: `class impClass; impClass& imp;`). Use it when you need to hide platform/3rd-party details.

### Error handling and logging
- No empty `catch {}` blocks.
- Prefer explicit failure modes:
  - Programmer errors (misuse): may throw (e.g. `runtime_error("...未初始化")`).
  - Runtime/hardware failures: often return error codes and/or log.
- Logging utilities:
  - `loong_utility-main/src/log.*` wraps quill.
  - Many run scripts capture stdout via `tee` into `../log/terminal_*.txt`.

## Safety / operational notes
- Some executables/scripts are intended for target hardware and may require `sudo` and real-time privileges.
- Be cautious running scripts that delete logs (`rm`/`sudo rm`) in automation.
- Avoid changing bus mappings / motor outputs without explicit user request.

## Third-party code
- Treat `*/third_party/` as vendored; avoid editing unless explicitly requested.
