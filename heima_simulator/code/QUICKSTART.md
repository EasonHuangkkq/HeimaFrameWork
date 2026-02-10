# Quick Start Guide

## Step 1: Export and Convert Model

1. **Export policy from training** (if not already done):
   ```bash
   cd /home/yao/Desktop/repo/heima/legged_gym
   # Edit legged_gym/scripts/play.py, set EXPORT_POLICY = True
   python legged_gym/scripts/play.py --task=heima_noarm
   ```
   This creates: `logs/rough_heima_noarm/exported/policies/policy_1.pt`

2. **Convert to ONNX**:
   ```bash
   cd /home/yao/Desktop/repo/heima_deploy/HeimaFrameWork/heima_rl_deploy
   python convert_pytorch_to_onnx.py \
       --input /home/yao/Desktop/repo/heima/legged_gym/logs/rough_heima_noarm/exported/policies/policy_1.pt \
       --output policy_heima_noarm.onnx
   ```

## Step 2: Build

```bash
cd /home/yao/Desktop/repo/heima_deploy/HeimaFrameWork/heima_rl_deploy
./build.sh
```

Or manually:
```bash
mkdir build && cd build
cmake ..
make
```

## Step 3: Prepare Configuration

Ensure `config.xml` is in the build directory or specify the path when running.

## Step 4: Run

```bash
cd build
./heima_rl_deploy policy_heima_noarm.onnx ../heimaSDK/config.xml 0.0 0.0 0.0
```

**Arguments**:
- `policy_heima_noarm.onnx`: ONNX model file
- `../heimaSDK/config.xml`: SDK configuration
- `0.0 0.0 0.0`: Command velocities [vx, vy, wz]

## Important Notes

1. **Base Linear Velocity**: Currently set to zero. For better performance, implement odometry estimation.

2. **Motor Mapping**: Ensure motors 0-11 in `config.xml` are leg joints in the correct order.

3. **Safety**: The program will:
   - Wait for motors to reach operational state
   - Gracefully disable motors on Ctrl+C
   - Hold current position on shutdown

4. **Control Frequency**: Runs at 1kHz (1ms loop). Ensure your system can maintain this rate.

## Troubleshooting

- **Library not found**: Set `LD_LIBRARY_PATH` to include ONNX Runtime library path
- **Model size mismatch**: Verify model expects 48-dim input and 12-dim output
- **Motor errors**: Check motor status words and error codes in output

