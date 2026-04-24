# AGENTS.md — loong_ctrl_locomotion-main

Locomotion algorithm library. This directory produces `libnabo_*` and follows a strict FSM + plan architecture built around isolation between plans.

## Structure

```text
loong_ctrl_locomotion-main/
├── src/manager/   # Nabo entry, FSM, plan scheduling
├── src/plan/      # concrete plans
├── src/robot/     # robot config / model-specific data
├── src/common/    # reusable algorithm helpers
├── src/rl/        # ONNX / RL-related code
├── test/          # simple example executable
└── tools/         # required build/run wrappers
```

## Local architecture rules

- The readme’s core principle is `隔离`:
  - new functionality should be added with minimum scope
  - top-level scheduling happens in `fsm`
  - the largest extension unit is a `plan`
  - plans own their submodules and remain independent from one another
- Shared modules/data reused by multiple plans are expected to be singleton-style rather than loosely duplicated.
- Plan lifecycle terminology used here: `init`, `hey`, `run`, `bye`, `log`.

## Build / run

Run wrappers from `tools/` only:

```bash
cd tools
./make.sh xs   # x64 sim
./make.sh xr   # x64 robot / co-sim
./make.sh as   # aarch64 sim
./make.sh ar   # aarch64 robot
./run_test.sh
```

Manual equivalent for the common sim build:

```bash
mkdir -p build_x64_sim && cd build_x64_sim
cmake .. -Dcpu_x64=1 -Dsim_mode=1 -Drl_use_cuda=0
make -j6 install
```

## Local gotchas

- CMake still expects `../loong_utility/loong_utility.cmake`; use the root symlink workaround if needed.
- `tools/make.sh` installs to `nabo_output/` and sibling projects consume the produced `libnabo_*`.
- `third_party/include/onnxruntime/` is vendored; do not patch it unless explicitly asked.
- `xr`/`ar` are not harmless variants; they represent robot-side builds and may change linked runtime behavior.

## Where to edit

| Task | Location |
|---|---|
| FSM / scheduling changes | `src/manager/` |
| New or changed plan behavior | `src/plan/` |
| Robot constants / config | `src/robot/` |
| ONNX / RL inference path | `src/rl/` |

## Validation

- Build the target variant you changed.
- For quick smoke validation, use `tools/run_test.sh` or run `build_*/main` directly.
- If public `nabo` headers or exported behavior changed, also verify at least one consumer (`loong_base-main` or `loong_sim-main`) still configures/builds.
