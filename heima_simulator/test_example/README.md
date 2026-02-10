# Minimal C++ gRPC Client Example

最小示例：从 C++ 调用 Python MuJoCo 服务器的 `_fetch_obs()` 和 `_write_torque()` 函数。

## 文件

- `mujoco_service.proto` - Protocol buffer 定义
- `grpc_client_minimal.cpp` - 最小 C++ 客户端
- `build.sh` - 构建脚本

## 构建

```bash
chmod +x build.sh
./build.sh
```

## 运行

### 1. 启动 Python 服务器（显示 MuJoCo 窗口）

```bash
cd /home/yao/Desktop/repo/heima/legged_gym/legged_gym/scripts/heima_simulator
python3 grpc_mujoco_server.py --config_file heima_noarm.yaml
```

### 2. 运行 C++ 客户端

```bash
cd /home/yao/Desktop/repo/heima/deploy_simulator/HeimaFrameWork/heima_simulator
./grpc_client_minimal
```

## 使用示例

```cpp
// 创建客户端
auto channel = grpc::CreateChannel("localhost:50051", 
                                   grpc::InsecureChannelCredentials());
MuJoCoClient client(channel);

// 获取观测
std::vector<float> base_ang_vel, joint_pos, joint_vel;
client.FetchObs(base_ang_vel, joint_pos, joint_vel);

// 发送力矩
std::vector<float> torques(12, 0.0f);
client.WriteTorque(torques);
```

## 依赖

- gRPC C++ 库
- Protocol Buffers 编译器
- pthread

Ubuntu/Debian 安装：
```bash
sudo apt install libgrpc++-dev libprotobuf-dev protobuf-compiler protobuf-compiler-grpc
```

如果遇到 `protoc: 未找到命令`，运行：
```bash
sudo apt install protobuf-compiler protobuf-compiler-grpc
```

