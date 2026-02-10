# Heima RL Deployment - Unified Simulator & Real Robot

This deployment program can run on both the MuJoCo simulator (via gRPC) and the real Heima robot (via DriverSDK).

## Features

- **Unified Interface**: Single codebase for both simulator and real robot
- **Mode Selection**: Choose between `sim` or `real` at runtime via command-line argument
- **Policy Deployment**: Runs ONNX neural network policy from legged_gym training
- **Real-time Control**: 50 Hz policy updates, 1 kHz communication loop

## Building

### Simulator Mode (Default)

```bash
cd /path/to/heima_simulator/code
mkdir build && cd build
cmake ..
make
```

This builds a **simulator-only** binary that can only run in `sim` mode.

### Real Robot Mode

To build with real robot support:

```bash
cd /path/to/heima_simulator/code
mkdir build && cd build
cmake -DBUILD_REAL_ROBOT=ON ..
make
```

This builds a binary that supports **both** `sim` and `real` modes.

**Note**: The DriverSDK will be built automatically from source. For EtherCAT support, install:
```bash
sudo apt-get install libethercat-dev
```

### CMake Options

- `BUILD_REAL_ROBOT=OFF` (default): Build simulator-only binary
- `BUILD_REAL_ROBOT=ON`: Build with real robot support (requires DriverSDK)

## Usage

### Command Line Arguments

```bash
./heima_sim_deploy [mode] [onnx_file] [config] [vx] [vy] [wz]
```

**Arguments**:
- `mode`: Robot mode - `"sim"` (simulator) or `"real"` (real robot). Default: `"sim"`
- `onnx_file`: Path to ONNX policy file. Default: `"policy_heima_noarm.onnx"`
- `config`: 
  - For `sim` mode: gRPC server address. Default: `"localhost:50051"`
  - For `real` mode: SDK config file path. Default: `"config.xml"`
- `vx`: Forward velocity command (m/s). Default: `0.0`
- `vy`: Lateral velocity command (m/s). Default: `0.0`
- `wz`: Yaw rate command (rad/s). Default: `0.0`

### Example Usage

#### Simulator Mode (default)

```bash
# Run with default settings (zero velocity)
./heima_sim_deploy

# Run with forward velocity 0.5 m/s
./heima_sim_deploy sim policy.onnx localhost:50051 0.5 0.0 0.0

# Run with custom gRPC server
./heima_sim_deploy sim policy.onnx 192.168.1.100:50051 0.3 0.0 0.0
```

#### Real Robot Mode

```bash
# Run on real robot with default config
./heima_sim_deploy real policy.onnx config.xml 0.0 0.0 0.0

# Run with forward velocity
./heima_sim_deploy real policy.onnx config.xml 0.5 0.0 0.0

# Run with custom config file
./heima_sim_deploy real policy.onnx /path/to/robot_config.xml 0.3 0.0 0.0
```

## Architecture

### Robot Interface Abstraction

The code uses an abstract `RobotInterface` class with two implementations:

1. **SimulatorInterface** (`robot_interface_sim.cpp`)
   - Communicates with MuJoCo simulator via gRPC
   - Fetches: RPY, angular velocity, joint positions/velocities
   - Sends: Joint torques

2. **RealRobotInterface** (`robot_interface_real.cpp`)
   - Communicates with real robot via DriverSDK
   - Fetches: IMU data, motor states
   - Sends: Motor torque commands

### Control Loop

- **Policy frequency**: 50 Hz (20 ms period)
- **Communication frequency**: 1 kHz (1 ms period)
- **PD control recalculation**: 200 Hz (5 ms period)

### Observation Vector (45 dimensions)

1. Base angular velocity (3) - scaled by 0.25
2. Projected gravity (3)
3. Commands [vx, vy, wz] (3) - scaled by [2.0, 2.0, 0.25]
4. Joint positions relative to default (12) - scaled by 1.0
5. Joint velocities (12) - scaled by 0.05
6. Previous actions (12)

## Safety Features

- **Torque limiting**: Maximum torque per motor configurable
- **Graceful shutdown**: Sends zero torques on exit
- **Signal handling**: Ctrl+C stops the robot safely
- **Motor status monitoring**: Waits for motors to be operational (real robot mode)

## Configuration Parameters

See `main.cpp` for:
- `DEFAULT_JOINT_ANGLES`: Default standing pose
- `PD_KP`, `PD_KD`: PD controller gains
- `MAX_TORQUE_LIMIT`: Safety torque limits
- `ACTION_SCALE`: Scaling factor for policy output

**Important**: These parameters should match the training configuration!

## Troubleshooting

### Simulator mode issues

- **"Failed to connect to gRPC server"**: Ensure MuJoCo simulator is running with gRPC server enabled
- Check gRPC server address and port

### Real robot mode issues

- **"DriverSDK not found"**: Ensure DriverSDK is built and available at `../../heimaSDK_copy/build`
- **"Not all motors reached operational state"**: Check motor connections and EtherCAT bus
- **Config file not found**: Ensure `config.xml` exists with correct motor configuration

### Build issues

- **"driver_sdk library not found"**: Build DriverSDK first
- **"gRPC library not found"**: Install libgrpc++-dev and libprotobuf-dev

## Files

- `main.cpp`: Main deployment program
- `robot_interface.h/cpp`: Abstract robot interface
- `robot_interface_sim.h/cpp`: Simulator implementation
- `robot_interface_real.h/cpp`: Real robot implementation
- `onnx_wrapper.h/cpp`: ONNX Runtime wrapper
- `ankle_solver/`: Ankle kinematics solver
- `proto/`: gRPC protocol buffers for simulator communication

## Related Documentation

- Training configuration: `/home/yao/Desktop/repo/heima/legged_gym/legged_gym/envs/heima_noarm/heima_noarm_config.py`
- Deployment YAML config: `/home/yao/Desktop/repo/heima/legged_gym/deploy/deploy_mujoco/configs/heima_noarm.yaml`
- Real robot deployment reference: `/home/yao/Desktop/repo/heima/deploy_simulator/HeimaFrameWork/heima_rl_deploy/main.cpp`
