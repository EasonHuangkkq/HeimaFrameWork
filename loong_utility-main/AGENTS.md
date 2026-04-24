# AGENTS.md — loong_utility-main

Shared utility library used by most C++ projects here. Keep changes small and API-stable because this directory fans out into locomotion, base, and sim builds.

## Structure

```text
loong_utility-main/
├── src/                 # actual utility code
├── loong_utility.cmake  # imported by sibling projects
├── 代码规范new.md        # canonical style doc copy
└── third_party/         # vendored deps
```

## Where to look

| Task | Location | Notes |
|---|---|---|
| Math helpers | `src/algorithms.*`, `src/pInv.h` | used from control code |
| Eigen aliases | `src/eigen.h` | project-preferred vector/matrix aliases |
| INI parsing | `src/ini.*` | runtime config loading |
| Logging | `src/log.*` | wraps quill |
| UDP | `src/udp.*` | shared transport helper |
| Timing | `src/timing.*` | sleep/time helpers |

## Local rules

- Keep public headers lightweight; this repo explicitly discourages pulling uncommon dependencies into `.h` files.
- Prefer existing aliases from `src/eigen.h` (`vec3f`, `mat3d`, etc.) over ad-hoc Eigen typedefs.
- `loong_utility.cmake` is part of the public contract for sibling projects; do not casually rename/move files it exports.
- `src/ini.*` is for config loading, not for hot real-time loops.
- `third_party/` is vendored. Avoid edits there.

## Module map

- `Alg::` = clip / threshold / filter helpers
- `Ei::` = Eigen aliases + math helpers
- `Ini::` = ini parsing
- `Log::` = quill-backed logging
- `Udp::` = UDP sockets
- `Timing::` = sleep / timing helpers

Preserve existing namespace boundaries instead of merging helpers into global utilities.

## Build / usage

- This directory is normally consumed via `include(../loong_utility/loong_utility.cmake)` from sibling projects.
- In this flattened workspace you may need the root symlink:
  - `ln -s loong_utility-main loong_utility`

## Notes

- There is no dedicated standalone test suite here.
- If a change affects headers in `src/`, verify at least one dependent project still builds.
