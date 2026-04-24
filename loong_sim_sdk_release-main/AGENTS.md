# AGENTS.md — loong_sim_sdk_release-main

Prebuilt full-chain simulation/runtime package. Treat this directory primarily as runtime artifacts, configs, Python SDK, and launch scripts — not as a normal source project.

## Structure

```text
loong_sim_sdk_release-main/
├── bin/      # prebuilt executables
├── config/   # runtime ini files
├── model/    # models, assets, onnx
├── module/   # prebuilt algorithm libs
├── sdk/      # Python SDK packages
├── py_jnt/   # joint-control examples
├── py_mani/  # manipulation examples
└── tools/    # launch scripts + UI
```

## Local rules

- Prefer editing configs, Python examples, or launch wrappers here — not prebuilt binaries.
- `bin/`, `module/`, and much of `model/` are release artifacts; treat them as generated/runtime payload unless explicitly asked otherwise.
- This project documents the real startup sequence for full-chain simulation; do not simplify it in docs unless you verify the full workflow.

## Run flow

All scripted operations happen in `tools/`:

```bash
./run_sim.sh
./run_driver.sh
./run_interface.sh
./run_locomotion.sh    # joint SDK path
./run_manipulation.sh  # manipulation SDK path
./py_ui.py
```

The readme describes ordered startup and control-state transitions; use it when documenting user flows.

## Where to look

| Task | Location |
|---|---|
| Startup sequence / operator flow | `readme.md`, `tools/` |
| Runtime config | `config/` |
| Python SDK internals | `sdk/` |
| Example scripts | `py_jnt/`, `py_mani/` |
| Release executables | `bin/` |

## Validation

- Validate config or script changes by following the documented startup order.
- If you change SDK/example code, verify against the corresponding joint or manipulation workflow from the readme.

## Notes

- This directory is the full-chain counterpart to `loong_sim-main`; keep those two mental models separate.
- Several scripts remove old log/shared-memory files before launch.
