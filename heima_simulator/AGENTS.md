# AGENTS.md — heima_simulator

Unified RL deployment project for simulator and real robot modes. Most meaningful source work happens under `code/`; other folders are helpers, experiments, or generated/test material.

## Structure

```text
heima_simulator/
├── code/           # main deploy project
├── test_example/   # gRPC/protobuf example + generated files
├── scripts/        # helper scripts
├── ankle_visualizer/
└── funny/          # ad-hoc experiments
```

## Focus area

- Primary engineering target: `code/`
- Inside `code/`:
  - `main.cpp` = main deployment entry
  - `robot_interface_*` = sim/real abstraction
  - `onnx_wrapper.*` = inference wrapper
  - `ankle_solver/` = kinematics helper
  - `proto/` = generated gRPC protobuf artifacts

## Build / run

Common path:

```bash
cd code
mkdir -p build && cd build
cmake ..
make
```

Real-robot-capable build:

```bash
cmake -DBUILD_REAL_ROBOT=ON ..
make
```

## Local rules

- `README.md` under `code/` is the reliable workflow doc for this project.
- `code/QUICKSTART.md` appears copied from `heima_rl_deploy/`; do not trust it over `code/README.md` without verification.
- Generated protobuf files under `code/proto/` and `test_example/` should be treated as generated artifacts.
- `BUILD_REAL_ROBOT=ON` pulls in DriverSDK/EtherCAT concerns; do not treat that build like a harmless pure-sim variant.

## Where to edit

| Task | Location |
|---|---|
| sim vs real abstraction | `code/robot_interface*.{h,cpp}` |
| policy loop / safety params | `code/main.cpp` |
| ONNX integration | `code/onnx_wrapper.*` |
| gRPC protocol | `code/proto/` (regenerate, don’t hand-edit generated files) |

## Validation

- Rebuild the exact mode you changed (`sim` only vs `BUILD_REAL_ROBOT=ON`).
- For logic changes, prefer validating in simulator mode first.
