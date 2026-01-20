#!/bin/bash
# Build script for Heima RL Deployment

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "=== Building Heima RL Deployment ==="

# Create build directory
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure
echo "Configuring..."
cmake ..

# Build
echo "Building..."
make -j$(nproc)

echo ""
echo "✓ Build complete!"
echo "  Executable: ${BUILD_DIR}/heima_rl_deploy"
echo ""
echo "To run:"
echo "  cd ${BUILD_DIR}"
echo "  ./heima_rl_deploy [onnx_file] [config_file] [vx] [vy] [wz]"

