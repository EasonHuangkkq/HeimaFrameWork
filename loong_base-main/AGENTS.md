# AGENTS.md — loong_base-main

Multi-process runtime layer. This directory builds the task executables (`driver`, `locomotion`, `interface`, `test`) and glues together DriverSDK, locomotion, shared memory, and UDP I/O.

## Structure

```text
loong_base-main/
├── main/driver/       # driver process entry
├── main/locomotion/   # locomotion process entry
├── main/interface/    # UDP / external command entry
├── main/test/         # lightweight test app
├── src/task_base/     # task base + DataCenter
├── src/common/        # shareMemClass and common runtime pieces
├── config/            # runtime xml/ini config
├── module/            # prebuilt runtime libraries
└── tools/             # required build/run wrappers
```

## Where to look

| Task | Location | Notes |
|---|---|---|
| Process bootstrapping | `main/*/main.cpp` | each executable is separate |
| Driver loop | `main/driver/` | interacts with DriverSDK |
| Locomotion loop | `main/locomotion/` | runs Nabo |
| Interface / UDP | `main/interface/` | external command bridge |
| Shared-memory registration | `src/task_base/data_center.*` | central runtime hub |
| Shared-memory implementation | `src/common/share_mem.*` | continuity/alignment matter |

## Build / run

Run wrappers from `tools/` only:

```bash
cd tools
./make.sh xs|xr|as|ar
./run_driver.sh [s]
./run_locomotion.sh [s]
./run_interface.sh [s]
./run_test.sh
```

Manual common build:

```bash
mkdir -p build_x64_sim && cd build_x64_sim
cmake .. -Dcpu_x64=1 -Dsim_mode=1
make -j6 install
```

## Local gotchas

- CMake still expects `../loong_utility/loong_utility.cmake`; use the root symlink workaround if configure fails.
- Runtime depends on prebuilt libs under `module/` and local config under `config/` (`driver.ini`, `thread.ini`, `config.xml`).
- `CMakeLists.txt` still references `nabo_manipulation` in rpath even though that module is not present here; do not “fix” this casually unless the user asks.
- Several run scripts remove shared-memory/log files before launch; inspect scripts before automating them.

## Data handling rules

- `share_mem.h` explicitly warns that exchanged structures must stay memory-contiguous.
- Avoid slipping `std::vector` or other non-contiguous members into shared-memory payload structs.
- If you touch `DataCenter` or shared-memory structs, verify both writer and reader sides.

## Validation

- Rebuild the relevant variant with `tools/make.sh ...`.
- If you changed a task executable, run its wrapper in the safest mode available (`s` for sim where applicable).
- If you changed shared-memory or interface data structures, validate both producer and consumer codepaths.
