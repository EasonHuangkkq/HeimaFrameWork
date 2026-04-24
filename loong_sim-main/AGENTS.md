# AGENTS.md — loong_sim-main

Algorithm-only MuJoCo simulation. This directory directly links the locomotion library into a simulator app; it is not the same thing as the full-chain runtime package in `loong_sim_sdk_release-main/`.

## Structure

```text
loong_sim-main/
├── main/ctrl_sim_loco/  # main executable entry
├── src/                 # MuJoCo / GLFW wrapper
├── model/               # xml, meshes, assets, onnx
├── config/              # sim/runtime ini files
├── module/              # prebuilt locomotion lib
└── tools/               # build + run scripts
```

## Local rules

- Keep the distinction clear:
  - `loong_sim-main` = direct algorithm simulation
  - `loong_sim_sdk_release-main` = full-chain runtime simulation
- Model/config paths are part of runtime behavior; avoid moving files casually.
- `third_party/` is vendored.

## Build / run

Use the wrapper scripts from `tools/`:

```bash
cd tools
./make.sh
./run_mujoco_loco.sh [delay_ms]
```

Manual equivalent:

```bash
mkdir -p build && cd build
cmake ..
make -j4 install
```

## Gotchas

- First-time Linux builds may need `libgl-dev`.
- `CMakeLists.txt` still includes `../loong_utility/loong_utility.cmake`; use the root symlink workaround if needed.
- `tools/clear_log.sh` and related wrappers delete log files; read them before using in automation.

## Where to edit

| Task | Location |
|---|---|
| MuJoCo / GLFW wrapper | `src/sim.*` |
| Main sim loop | `main/ctrl_sim_loco/main.cpp` |
| Model assets / xml | `model/` |
| Runtime config | `config/` |

## Validation

- Rebuild with `tools/make.sh`.
- Smoke-test with `tools/run_mujoco_loco.sh` when runtime deps are available.
