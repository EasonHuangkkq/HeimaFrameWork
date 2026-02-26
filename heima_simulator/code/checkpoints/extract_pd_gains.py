#!/usr/bin/env python3
"""
Extract PD controller gains and other metadata from ONNX file and write to config.yaml.

Usage:
    python extract_pd_gains.py --onnx <onnx_file> [--output <config.yaml>] [--checkpoint <checkpoint_dir>]
"""

import onnx
import argparse
import os
from pathlib import Path


def parse_csv_string(csv_str: str) -> list:
    """Parse comma-separated string to list of floats."""
    return [float(x.strip()) for x in csv_str.split(',')]


def convert_onnx_to_checkpoint_order(values: list) -> list:
    """
    Convert from ONNX order [right_leg_6, left_leg_6] to checkpoint order [left_leg_6, right_leg_6, waist_3].
    
    ONNX order: [right_6_joints, left_6_joints] = 12 values
    Checkpoint order: [left_6_joints, right_6_joints, waist_3] = 15 values
    """
    if len(values) != 12:
        raise ValueError(f"Expected 12 values, got {len(values)}")
    
    right_leg = values[:6]
    left_leg = values[6:12]
    
    # Checkpoint order: [left_leg, right_leg, waist]
    # Add default waist values (will be overridden if waist data exists)
    return left_leg + right_leg + [0.0, 0.0, 0.0]


def extract_metadata_from_onnx(onnx_path: str) -> dict:
    """Extract all metadata from ONNX file."""
    model = onnx.load(onnx_path)
    
    # Convert metadata_props to dictionary
    metadata = {prop.key: prop.value for prop in model.metadata_props}
    
    result = {}
    
    # Extract joint names
    if 'joint_names' in metadata:
        joint_names = [x.strip() for x in metadata['joint_names'].split(',')]
        result['joint_names'] = joint_names
        print(f"Found {len(joint_names)} joints: {joint_names}")
    
    # Extract kp (joint_stiffness)
    if 'joint_stiffness' in metadata:
        kp = parse_csv_string(metadata['joint_stiffness'])
        result['kp'] = kp
        print(f"Extracted kp (stiffness): {kp}")
    else:
        raise ValueError("ONNX file does not contain 'joint_stiffness' metadata")
    
    # Extract kd (joint_damping)
    if 'joint_damping' in metadata:
        kd = parse_csv_string(metadata['joint_damping'])
        result['kd'] = kd
        print(f"Extracted kd (damping): {kd}")
    else:
        raise ValueError("ONNX file does not contain 'joint_damping' metadata")
    
    # Extract default_joint_pos
    if 'default_joint_pos' in metadata:
        default_pos = parse_csv_string(metadata['default_joint_pos'])
        result['default_joint_pos'] = default_pos
        print(f"Extracted default_joint_pos: {default_pos}")
    
    # Extract action_scale
    if 'action_scale' in metadata:
        action_scale = parse_csv_string(metadata['action_scale'])
        result['action_scale'] = action_scale
        print(f"Extracted action_scale: {action_scale}")
    
    # Extract other metadata
    if 'run_path' in metadata:
        result['run_path'] = metadata['run_path']
    if 'command_names' in metadata:
        result['command_names'] = metadata['command_names']
    if 'observation_names' in metadata:
        result['observation_names'] = metadata['observation_names']
    
    return result


def write_config_yaml(output_path: str, metadata: dict):
    """Write extracted metadata to config.yaml file."""
    
    # Convert to checkpoint order
    kp = convert_onnx_to_checkpoint_order(metadata['kp'])
    kd = convert_onnx_to_checkpoint_order(metadata['kd'])
    
    # Override waist values with defaults
    kp[12:15] = [200.0, 200.0, 200.0]  # Waist kp
    kd[12:15] = [1.0, 1.0, 1.0]  # Waist kd
    
    # Convert default_joint_pos if available
    if 'default_joint_pos' in metadata:
        default_angles = convert_onnx_to_checkpoint_order(metadata['default_joint_pos'])
        default_angles[12:15] = [0.0, 0.0, 0.0]  # Waist default angles
    else:
        default_angles = [0.0] * 15
    
    # Convert action_scale if available
    if 'action_scale' in metadata:
        action_scale = convert_onnx_to_checkpoint_order(metadata['action_scale'])
        action_scale[12:15] = [1.0, 1.0, 1.0]  # Waist action scale
    else:
        action_scale = [1.0] * 15
    
    # Joint names for comments
    joint_names = [
        "Left: hip_roll", "Left: hip_yaw", "Left: hip_pitch", "Left: knee",
        "Left: ankle_pitch", "Left: ankle_roll",
        "Right: hip_roll", "Right: hip_yaw", "Right: hip_pitch", "Right: knee",
        "Right: ankle_pitch", "Right: ankle_roll",
        "Waist motor 1", "Waist motor 2", "Waist motor 3"
    ]
    
    with open(output_path, 'w') as f:
        f.write("# Heima Robot Control Configuration\n")
        f.write("# PD Controller Gains, Default Joint Angles, and Action Scales\n")
        f.write("# Extracted from ONNX model metadata\n\n")
        
        # PD Gains
        f.write("# PD Controller Gains (stiffness = kp, damping = kd)\n")
        f.write("# Order: Left leg [hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll]\n")
        f.write("#        Right leg [hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll]\n")
        f.write("#        Waist [3 motors]\n")
        f.write("pd_gains:\n")
        f.write("  kp:  # Stiffness (N·m/rad)\n")
        for val, name in zip(kp, joint_names):
            f.write(f"    - {val}  # {name}\n")
        f.write("  kd:  # Damping (N·m·s/rad)\n")
        for val, name in zip(kd, joint_names):
            f.write(f"    - {val}  # {name}\n")
        f.write("\n")
        
        # Default joint angles
        f.write("# Default joint angles (radians) when action = 0.0\n")
        f.write("# Order: Left leg [hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll]\n")
        f.write("#        Right leg [hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll]\n")
        f.write("#        Waist [3 motors]\n")
        f.write("default_joint_angles:\n")
        for val, name in zip(default_angles, joint_names):
            f.write(f"  - {val}  # {name}\n")
        f.write("\n")
        
        # Action scales
        f.write("# Action scales (per-joint scales for neural network actions)\n")
        f.write("# Order: Same as default_joint_angles\n")
        f.write("# Action is applied as: target_angle = default_angle + action * action_scale\n")
        f.write("action_scale:\n")
        for val, name in zip(action_scale, joint_names):
            f.write(f"  - {val}  # {name}\n")
        f.write("\n")
        
        # Observation scales (default values, not in ONNX metadata)
        f.write("# Observation scaling factors (for neural network input normalization)\n")
        f.write("# These scales are applied to observations before feeding to the neural network\n")
        f.write("observation_scales:\n")
        f.write("  lin_vel_scale: 2.0      # Linear velocity scale (m/s)\n")
        f.write("  ang_vel_scale: 0.25    # Angular velocity scale (rad/s)\n")
        f.write("  dof_pos_scale: 1.0     # Joint position scale (rad)\n")
        f.write("  dof_vel_scale: 0.05    # Joint velocity scale (rad/s)\n")
        f.write("  cmd_scale:              # Command scale [vx, vy, wz]\n")
        f.write("    - 2.0                 # vx (forward/backward)\n")
        f.write("    - 2.0                 # vy (left/right)\n")
        f.write("    - 0.25                # wz (yaw rate)\n")
    
    print(f"\n✓ Successfully wrote config to: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract PD gains and metadata from ONNX file to config.yaml")
    parser.add_argument("--onnx", type=str, required=True, help="Path to ONNX model file")
    parser.add_argument("--output", type=str, help="Output config.yaml path (default: same dir as ONNX or checkpoint dir)")
    parser.add_argument("--checkpoint", type=str, help="Checkpoint directory name (e.g., v6). If provided, writes to checkpoint_dir/config.yaml")
    args = parser.parse_args()
    
    # Determine output path
    if args.checkpoint:
        script_dir = Path(__file__).parent
        checkpoint_dir = script_dir / args.checkpoint
        if not checkpoint_dir.exists():
            print(f"Error: Checkpoint directory {checkpoint_dir} does not exist")
            exit(1)
        output_path = checkpoint_dir / "config.yaml"
    elif args.output:
        output_path = Path(args.output)
    else:
        # Default: same directory as ONNX file
        onnx_path = Path(args.onnx)
        output_path = onnx_path.parent / "config.yaml"
    
    # Extract metadata
    print(f"Loading ONNX file: {args.onnx}")
    metadata = extract_metadata_from_onnx(args.onnx)
    
    # Write config
    print(f"\nWriting config to: {output_path}")
    write_config_yaml(str(output_path), metadata)