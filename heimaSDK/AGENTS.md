# AGENTS.md — heimaSDK

Hardware-facing DriverSDK source tree: EtherCAT + CAN + RS232/RS485 + IMU + demos. Treat this directory as safety-critical.

More specific child guidance exists under `web_control/AGENTS.md` for the browser-control subtree.

## Structure

```text
heimaSDK/
├── heima_driver_sdk.*   # top-level SDK API / orchestration
├── ecat.*               # EtherCAT master, domains, RT loop
├── common.* ptr_que.h   # triple-buffer + shared data plumbing
├── config_xml.*         # config.xml parsing / alias-domain maps
├── rs232.* rs485.* can.*
├── yesense*/            # IMU integration
├── docs/ DATAFLOW.md    # architecture notes worth reading first
├── web_control/         # separate web-control side path
└── *_test.cpp *_demo.cpp# hardware smoke tests / demos
```

## Where to look

| Task | Location | Notes |
|---|---|---|
| SDK public API | `heima_driver_sdk.h/.cpp` | orchestration entrypoint |
| EtherCAT domains / RT loop | `ecat.h/.cpp` | core safety-sensitive logic |
| Triple buffer / wrappers | `common.h/.cpp`, `ptr_que.h` | read these before touching data flow |
| XML topology parsing | `config_xml.*`, `config*.xml` | alias/type/domain mapping |
| IMU | `yesense_imu.*`, `yesense/` | serial-based IMU path |
| Browser/web control | `web_control/` | separate workflow from core SDK |
| Architecture docs | `DATAFLOW.md`, `docs/*.md`, `框架结构.md` | often faster than reverse-engineering from code |

## Build / run

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target driver_sdk
cmake --build build --target loong_driver_sdk
cmake --build build --target leg_smoke_test
```

Also available:

```bash
./build.sh
CC=aarch64-linux-gnu-gcc ./build.sh
```

## Local rules

- Keep `config.xml` semantics aligned with the physical EtherCAT topology before powering hardware.
- Do not remap axes or change bus mappings while the system is running.
- Preserve packed/continuous data layouts and alignment assumptions; several docs call out continuity requirements explicitly.
- Read `DATAFLOW.md` / `docs/ECAT_DOMAIN_FLOW.md` before changing `ecat.cpp` or `common.cpp`.
- `tinyxml2.*` is vendored third-party code in-tree; avoid editing unless explicitly required.

## Validation

- There is no software-only unit suite here.
- Validation is by hardware smoke targets such as:
  - `leg_smoke_test`
  - `arm_smoke_test`
  - `whole_body_smoke_test`
  - `whole_body_pvt_test`
  - `test_ankle_hardware`
- Typical real checks also involve `ethercat slaves` / runtime console logs.

## Safety notes

- Many runs need target hardware, `/usr/local/bin/ethercat`, and real-time privileges.
- Some demos command motors directly; do not run them casually in automation.
- `build/` may contain generated outputs; prefer editing sources, not build artifacts.
