# AGENTS.md — heimaFrameWork workspace

Multi-project robotics workspace: hardware SDK, shared utility library, locomotion library, multi-process runtime, MuJoCo sim, prebuilt sim SDK, and two RL deployment projects.

## Structure

```text
./
├── heimaSDK/                  # DriverSDK source, demos, EtherCAT/serial/CAN
├── loong_utility-main/        # Shared utility library + style docs
├── loong_ctrl_locomotion-main/# Nabo locomotion library
├── loong_base-main/           # driver/locomotion/interface task executables
├── loong_sim-main/            # algorithm-only MuJoCo simulation
├── loong_sim_sdk_release-main/# prebuilt full-chain runtime package
├── heima_simulator/           # unified sim/real RL deployment via gRPC/DriverSDK
└── heima_rl_deploy/           # smaller standalone RL deploy tool
```

## Child AGENTS.md files

- `heimaSDK/AGENTS.md`
- `heimaSDK/web_control/AGENTS.md`
- `loong_utility-main/AGENTS.md`
- `loong_ctrl_locomotion-main/AGENTS.md`
- `loong_base-main/AGENTS.md`
- `loong_sim-main/AGENTS.md`
- `loong_sim_sdk_release-main/AGENTS.md`
- `heima_simulator/AGENTS.md`
- `heima_rl_deploy/AGENTS.md`

Read the nearest child file before changing code there. Keep root rules here focused on cross-workspace behavior only.

## Where to look

| Task | Location | Notes |
|---|---|---|
| DriverSDK / bus logic | `heimaSDK/` | Hardware-facing, real-time, demos mixed with library sources |
| Shared math/log/udp helpers | `loong_utility-main/src/` | Used by most C++ subprojects |
| Locomotion library | `loong_ctrl_locomotion-main/src/` | FSM + plan architecture, produces `libnabo_*` |
| Multi-process runtime | `loong_base-main/` | Shared memory + `driver/interface/locomotion` executables |
| Algorithm sim | `loong_sim-main/` | Directly links locomotion library into MuJoCo app |
| Full-chain sim runtime | `loong_sim_sdk_release-main/` | Mostly prebuilt binaries + configs + Python SDK |
| RL sim/real deploy | `heima_simulator/code/` | gRPC sim + real-robot mode in one codebase |
| Standalone RL deploy | `heima_rl_deploy/` | Smaller ONNX + DriverSDK entrypoint |

## Workspace rules

### Original repo names still leak into build scripts
- Many CMake files do `include(../loong_utility/loong_utility.cmake)`.
- This workspace directory is `loong_utility-main/`, so source builds may need:
  - `ln -s loong_utility-main loong_utility`
- Some helper scripts/docs also refer to `loong_sim/` or `loong_base/` without `-main`; prefer a symlink fix over rewriting scripts unless explicitly asked.

### Run wrapper scripts from `tools/`
- `loong_ctrl_locomotion-main/tools/make.sh`
- `loong_base-main/tools/make.sh`
- `loong_sim-main/tools/make.sh`
- most `run_*.sh` wrappers in `loong_base-main/`, `loong_sim-main/`, and `loong_sim_sdk_release-main/`

These wrappers rely on `cd ..` and relative paths.

### Style docs are shared across projects
- Canonical project style docs live in:
  - `heimaSDK/代码规范.md`
  - `loong_utility-main/代码规范new.md`
- Core rules repeated across the workspace:
  - braces stay on the same line
  - `if(`/`for(`/function names do not get a space before `(`
  - always use braces for `if`/`for`/`while`
  - variable names are lowerCamelCase; files are lower_snake_case
  - keep uncommon dependencies out of headers; use pImpl when needed

### Units and frames
- Default units: SI
- Coordinate frame: z-up, x-forward, right-handed
- Euler order: zyx

## Build / test map

| Project | Build | Test / smoke run |
|---|---|---|
| `heimaSDK/` | `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release` | hardware smoke executables under `build/` |
| `loong_ctrl_locomotion-main/` | `cd tools && ./make.sh xs|xr|as|ar` | `cd tools && ./run_test.sh` or run `build_*/main` |
| `loong_base-main/` | `cd tools && ./make.sh xs|xr|as|ar` | `cd tools && ./run_test.sh` |
| `loong_sim-main/` | `cd tools && ./make.sh` | `cd tools && ./run_mujoco_loco.sh [delay_ms]` |
| `heima_simulator/code/` | `cmake ..` or `cmake -DBUILD_REAL_ROBOT=ON ..` | run `heima_sim_deploy sim|real ...` |
| `heima_rl_deploy/` | `./build.sh` or plain CMake | run built `heima_rl_deploy` manually |

There is no repo-wide CI or unified unit-test runner; validation is mostly CMake builds, example executables, simulation smoke runs, or hardware smoke tests.

## Safety / do-not-touch

- Treat `*/third_party/` as vendored unless explicitly asked.
- Treat generated protobuf files under `heima_simulator/**/proto/` and `heima_simulator/test_example/` as generated artifacts.
- Be careful with scripts that run `rm` or `sudo rm`, especially:
  - `loong_base-main/tools/clear_log.sh`
  - `loong_base-main/tools/safe_clear_log.sh`
  - `loong_sim-main/tools/clear_log.sh`
  - runtime wrappers that remove shared-memory/log files before launch
- Do not change motor/bus mappings, EtherCAT topology, or `config.xml` semantics unless the user explicitly requests it.

## Notes

- `loong_sim-main` is algorithm simulation; `loong_sim_sdk_release-main` is full-chain runtime simulation. Do not confuse them.
- `heimaSDK/` is the active SDK source directory in this workspace; some docs still mention `heimaSDK_copy/` from the original layout.
