# Repository Guidelines

## Project Structure & Module Organization
- Root holds the C/C++20 EtherCAT driver stack: `loong_driver_sdk.*` orchestrates buses, `ecat.*` wraps the EtherCAT master, `rs232.*` and `rs485.*` cover serial links, `config_xml.*` parses `config.xml` to build alias/type maps, `common.*` and `ptr_que.h` provide shared data structures, `tinyxml2.*` is vendored XML support, and `robot_multi_joint_api.c` offers a reference entrypoint.
- Build outputs land in `build/` (created by scripts); keep `config.xml` beside binaries or provide an absolute path when loading.

## Build, Test, and Development Commands
- `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release` then `cmake --build build --target driver_sdk` to produce `build/libdriver_sdk.so` (expects EtherCAT headers in `/usr/local/include` and `libethercat.so` in `/usr/local/lib`).
- `cmake --build build --target run_motor` builds the example executable if `main.cpp` is present.
- `./build.sh` compiles `robot_multi_joint_api.c` into `build/libethercat_robot.so`; override the compiler for cross-builds with `CC=aarch64-linux-gnu-gcc ./build.sh`.
- Runtime assumes `/usr/local/bin/ethercat` utilities are installed; run on target hardware with real-time privileges (often `sudo`).

## Coding Style & Naming Conventions
- 4-space indentation, brace on the same line, and `namespace DriverSDK` wraps C++ modules.
- Types and classes use PascalCase (`ECAT`, `SwapList`); C++ methods lean camelCase, while C functions stay snake_case; constants/macros are ALL_CAPS.
- Preserve packed struct layouts and alignment notes; keep Apache-2.0 headers intact.

## Testing Guidelines
- No automated test suite is present; rely on hardware-in-loop smoke tests.
- Typical check: `sudo ./build/run_motor` or a consumer of `libdriver_sdk.so`, while monitoring `ethercat slaves` for link/device state.
- Validate `config.xml` variants before deployment and watch console logs for PDO offset prints or SDO/reg request warnings.

## Commit & Pull Request Guidelines
- Git history is not available here; use concise Conventional Commit-style subjects (e.g., `fix: guard PDO writes during init`).
- In PRs, describe the hardware setup, expected PDO map, and reproduction steps; attach relevant logs or `ethercat slaves` output when touching bus logic.
- Call out any ABI changes in public headers and link related issues/task IDs.

## Configuration & Safety
- `config.xml` governs alias/type mapping; keep IDs in sync with the actual EtherCAT topology before powering motors.
- Do not remap axes while the system is running (see safeguards in `robot_multi_joint_api.c`); apply mapping changes offline and restart the stack.
