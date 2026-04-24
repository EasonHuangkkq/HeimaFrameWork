# AGENTS.md — heimaSDK/web_control

Browser-control side path for the SDK. This is a mixed C++ + Python subdomain with its own build and runtime flow, separate from the core EtherCAT SDK loop.

## Structure

```text
web_control/
├── whole_body_web_control.cpp
├── web_server.py
├── run_server.sh
├── requirements.txt
├── static/
└── CMakeLists.txt
```

## Build / run

Build the C++ backend from this directory:

```bash
cmake -S . -B build
cmake --build build
```

Run the web server with:

```bash
./run_server.sh
```

That script manages a Python virtualenv and launches the web app.

## Local rules

- Treat this as a separate control surface, not the canonical core SDK path.
- Python deps come from `requirements.txt`; avoid mixing them into unrelated workspace environments.
- `static/` holds web assets for the browser UI.
- Keep any hardware-command assumptions aligned with the parent `heimaSDK/` safety rules.

## Where to edit

| Task | Location |
|---|---|
| HTTP / WebSocket server logic | `web_server.py` |
| SDK-side control bridge | `whole_body_web_control.cpp` |
| front-end assets | `static/` |

## Validation

- Rebuild the local C++ target if you touched `whole_body_web_control.cpp`.
- Re-run `./run_server.sh` if you changed Python or asset behavior.
