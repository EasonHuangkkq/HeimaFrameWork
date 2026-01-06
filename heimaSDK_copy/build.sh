#!/bin/bash
#
# build.sh - Build shared library for robot2 EtherCAT interface
#
# Usage examples:
#   ./build.sh                               # build with default gcc
#   CC=aarch64-linux-gnu-gcc ./build.sh      # override compiler (cross-build)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

BUILD_DIR="${SCRIPT_DIR}/build"
mkdir -p "${BUILD_DIR}"

: "${CC:=gcc}"

INCLUDES=(
    "-I${REPO_ROOT}/include"
    "-I/usr/local/include"
)

LIB_DIRS=(
    "-L/usr/local/lib"
)

SRCS=(
    "${SCRIPT_DIR}/robot_multi_joint_api.c"
)
OUT="${BUILD_DIR}/libethercat_robot.so"

printf '[BUILD] CC=%s\n' "${CC}"
printf '[BUILD] Output: %s\n' "${OUT}"

"${CC}" -O2 -pipe -fPIC \
    "${INCLUDES[@]}" \
    "${SRCS[@]}" \
    -shared -Wl,-soname,libethercat_robot.so \
    "${LIB_DIRS[@]}" \
    -lethercat -lpthread -lm \
    -o "${OUT}"

printf '[DONE] Built %s\n' "${OUT}"
