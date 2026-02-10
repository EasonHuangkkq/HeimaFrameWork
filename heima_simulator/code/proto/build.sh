#!/bin/bash
# Minimal build script for gRPC client

set -e

echo "=== Building Minimal gRPC Client ==="

# 1. Generate C++ code from .proto
echo "1. Generating C++ protobuf files..."

# 首先尝试找到 grpc_cpp_plugin
GRPC_PLUGIN=$(which grpc_cpp_plugin 2>/dev/null)

if [ -z "$GRPC_PLUGIN" ]; then
    # 尝试其他常见位置
    if [ -f "/usr/local/bin/grpc_cpp_plugin" ]; then
        GRPC_PLUGIN="/usr/local/bin/grpc_cpp_plugin"
    elif [ -f "/usr/bin/grpc_cpp_plugin" ]; then
        GRPC_PLUGIN="/usr/bin/grpc_cpp_plugin"
    else
        echo "ERROR: grpc_cpp_plugin not found!"
        echo "Please install gRPC development tools:"
        echo "  sudo apt-get install libgrpc++-dev protobuf-compiler-grpc"
        echo "  OR build gRPC from source"
        exit 1
    fi
fi

echo "Using grpc_cpp_plugin: $GRPC_PLUGIN"

protoc -I. \
       --cpp_out=. \
       --grpc_out=. \
       --plugin=protoc-gen-grpc="$GRPC_PLUGIN" \
       mujoco_service.proto

# 2. Compile C++ client
# echo "2. Compiling C++ client..."
# g++ -std=c++17 \
#     grpc_client_minimal.cpp \
#     mujoco_service.pb.cc \
#     mujoco_service.grpc.pb.cc \
#     -o grpc_client_minimal \
#     -I/usr/local/include \
#     -I. \
#     -L/usr/local/lib \
#     -lgrpc++ \
#     -lprotobuf \
#     -lpthread

# echo ""
# echo "=== Build Complete ==="
# echo "Run: ./grpc_client_minimal"