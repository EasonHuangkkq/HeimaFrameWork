# Heima RL Deployment

This program deploys a trained neural network policy from `legged_gym` (heima_noarm) to control the robot using the Heima SDK.

## Overview

The deployment program:
- Loads an ONNX model converted from the trained PyTorch policy
- Reads motor states and IMU data from the SDK
- Converts sensor data to the 48-dimensional observation space used during training
- Runs neural network inference to compute motor actions
- Sends motor commands back through the SDK

## Prerequisites

1. **Trained Model**: Export your trained policy from legged_gym:
   ```bash
   cd /home/yao/Desktop/repo/heima/legged_gym
   python legged_gym/scripts/play.py --task=heima_noarm
   ```
   (Set `EXPORT_POLICY = True` in play.py)

2. **Convert to ONNX**: Use the provided conversion script:
   ```bash
   cd /home/yao/Desktop/repo/heima_deploy/HeimaFrameWork/heima_rl_deploy
   python convert_pytorch_to_onnx.py \
       --input /home/yao/Desktop/repo/heima/legged_gym/logs/rough_heima_noarm/exported/policies/policy_1.pt \
       --output policy_heima_noarm.onnx
   ```

3. **Dependencies**:
   - ONNX Runtime (from loong_ctrl_locomotion-main/third_party)
   - Heima SDK (from heimaSDK)
   - EtherCAT library
   - CMake 3.10+

## Building

```bash
cd /home/yao/Desktop/repo/heima_deploy/HeimaFrameWork/heima_rl_deploy
mkdir build
cd build
cmake ..
make
```

## Configuration

1. **SDK Config**: Ensure `config.xml` is in the working directory or specify path
2. **ONNX Model**: Place the converted ONNX model in the working directory

## Usage

### Basic Usage

```bash
./heima_rl_deploy [onnx_file] [config_file] [vx] [vy] [wz]
```

**Arguments**:
- `onnx_file`: Path to ONNX model (default: `policy_heima_noarm.onnx`)
- `config_file`: Path to SDK config XML (default: `config.xml`)
- `vx`: Command velocity X (m/s) (default: 0.0)
- `vy`: Command velocity Y (m/s) (default: 0.0)
- `wz`: Command angular velocity Z (rad/s) (default: 0.0)

### Examples

```bash
# Run with default settings (zero velocity commands)
./heima_rl_deploy

# Run with custom model and commands
./heima_rl_deploy policy_heima_noarm.onnx config.xml 0.5 0.0 0.0

# Forward motion
./heima_rl_deploy policy_heima_noarm.onnx config.xml 1.0 0.0 0.0

# Lateral motion
./heima_rl_deploy policy_heima_noarm.onnx config.xml 0.0 0.5 0.0

# Rotation
./heima_rl_deploy policy_heima_noarm.onnx config.xml 0.0 0.0 0.5
```

## Observation Space

The program converts SDK sensor data to match the training observation space (48 dimensions):

1. **Base linear velocity** (3) × 2.0
2. **Base angular velocity** (3) × 0.25
3. **Projected gravity** (3)
4. **Commands** (3) × [2.0, 2.0, 0.25]
5. **DOF positions** (12) - relative to default × 1.0
6. **DOF velocities** (12) × 0.05
7. **Previous actions** (12)

**Note**: Base linear velocity is currently set to zero. In a real deployment, you may need to:
- Use odometry from wheel encoders
- Estimate from IMU and joint velocities
- Use a state estimator (e.g., EKF)

## Action Space

The neural network outputs 12 actions (one per leg joint). Actions are:
- Relative to default joint angles
- Scaled by `ACTION_SCALE = 1.0`
- Converted to absolute joint positions: `target = default + action * scale`

## Default Joint Angles

From training configuration:
```cpp
Right leg: [0.0, 0.0, 0.2, -0.43, 0.22, 0.0]
Left leg:  [0.0, 0.0, 0.2, -0.43, 0.22, 0.0]
```

Order: hip_roll, hip_yaw, hip_pitch, knee_pitch, ankle_pitch, ankle_roll

## PD Control

The program uses PD gains from training:
- **Kp**: [300, 300, 300, 400, 120, 120] × 2 legs
- **Kd**: [1, 1, 1, 4, 1, 1] × 2 legs

These are applied at the driver level (configured in `config.xml`).

## Control Loop

The program runs at **1kHz** (1ms period):
1. Read motor states and IMU
2. Build observation vector
3. Run neural network inference
4. Convert actions to joint targets
5. Send motor commands
6. Advance SDK communication cycle

## Safety Features

- **Graceful shutdown**: On Ctrl+C, motors are disabled and held at current position
- **Motor state checking**: Waits for motors to reach operational state before starting
- **Error handling**: Checks for inference failures and motor errors

## Troubleshooting

### Model Input/Output Size Mismatch

Ensure the ONNX model expects:
- Input: 48 dimensions
- Output: 12 dimensions

Check with:
```python
import onnx
model = onnx.load("policy_heima_noarm.onnx")
print(model.graph.input[0])
print(model.graph.output[0])
```

### Base Linear Velocity

Currently set to zero. To improve performance:
1. Implement odometry estimation
2. Use a state estimator
3. Fuse IMU and joint velocity data

### Motor Mapping

Ensure motors 0-11 in your `config.xml` correspond to leg joints in the correct order:
- Right leg: hip_roll, hip_yaw, hip_pitch, knee_pitch, ankle_pitch, ankle_roll
- Left leg: hip_roll, hip_yaw, hip_pitch, knee_pitch, ankle_pitch, ankle_roll

### ONNX Runtime Library

If you get library loading errors:
1. Check that ONNX Runtime is in the correct path
2. Set `LD_LIBRARY_PATH`:
   ```bash
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/onnxruntime/lib
   ```

## File Structure

```
heima_rl_deploy/
├── main.cpp                 # Main deployment program
├── onnx_wrapper.h          # ONNX Runtime wrapper header
├── onnx_wrapper.cpp        # ONNX Runtime wrapper implementation
├── convert_pytorch_to_onnx.py  # Model conversion script
├── CMakeLists.txt          # Build configuration
└── README.md              # This file
```

## Integration with Training

The observation and action spaces match the training configuration in:
- `/home/yao/Desktop/repo/heima/legged_gym/legged_gym/envs/heima_noarm/heima_noarm_config.py`

Key parameters:
- `num_observations = 48`
- `num_actions = 12`
- `action_scale = 1.0`
- `obs_scales.lin_vel = 2.0`
- `obs_scales.ang_vel = 0.25`
- `obs_scales.dof_pos = 1.0`
- `obs_scales.dof_vel = 0.05`

## Notes

- The control loop runs at 1kHz for real-time performance
- Motor commands use position control (CSP mode)
- The program assumes the first 12 motors are leg joints
- Base linear velocity estimation is a TODO for improved performance

## See Also

- Heima SDK documentation: `../heimaSDK/README.md`
- Training repository: `/home/yao/Desktop/repo/heima/legged_gym`
- Deployment config: `../loong_sim_sdk_release-main/config/plan_rl.ini`

