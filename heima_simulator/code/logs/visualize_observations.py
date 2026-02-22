#!/usr/bin/env python3
"""
Visualize observations logged from the Heima deployment code.
Reads observations_log.csv and creates plots for analysis.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def main():
    # Read CSV file
    csv_file = "observations_log.csv"
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found")
        return
    
    print(f"Loading {csv_file}...")
    df = pd.read_csv(csv_file)
    print(f"Loaded {len(df)} samples")
    
    # Convert timestamp to seconds
    time_s = df['timestamp_ms'].values / 1000.0
    
    # Check if torque data is available
    has_torque = 'torque_0' in df.columns
    
    # Create figure with multiple subplots
    num_rows = 5 if has_torque else 4
    fig = plt.figure(figsize=(16, 3 * num_rows))
    
    # 1. Base orientation (Roll, Pitch, Yaw)
    ax1 = plt.subplot(num_rows, 2, 1)
    ax1.plot(time_s, df['roll'], label='Roll', linewidth=1.5)
    ax1.plot(time_s, df['pitch'], label='Pitch', linewidth=1.5)
    ax1.plot(time_s, df['yaw'], label='Yaw', linewidth=1.5)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (rad)')
    ax1.set_title('Base Orientation (RPY)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. Base angular velocity
    ax2 = plt.subplot(num_rows, 2, 2)
    ax2.plot(time_s, df['ang_vel_x'], label='ω_x', linewidth=1.5)
    ax2.plot(time_s, df['ang_vel_y'], label='ω_y', linewidth=1.5)
    ax2.plot(time_s, df['ang_vel_z'], label='ω_z', linewidth=1.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title('Base Angular Velocity')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. Joint positions (left leg)
    ax3 = plt.subplot(num_rows, 2, 3)
    joint_names_left = ['hip_roll', 'hip_yaw', 'hip_pitch', 'knee', 'ankle_pitch', 'ankle_roll']
    for i in range(6):
        ax3.plot(time_s, df[f'joint_pos_{i}'], label=joint_names_left[i], linewidth=1.0)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (rad)')
    ax3.set_title('Joint Positions - Left Leg')
    ax3.legend(fontsize=8, ncol=2)
    ax3.grid(True, alpha=0.3)
    
    # 4. Joint positions (right leg)
    ax4 = plt.subplot(num_rows, 2, 4)
    joint_names_right = ['hip_roll', 'hip_yaw', 'hip_pitch', 'knee', 'ankle_pitch', 'ankle_roll']
    for i in range(6, 12):
        ax4.plot(time_s, df[f'joint_pos_{i}'], label=joint_names_right[i-6], linewidth=1.0)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Position (rad)')
    ax4.set_title('Joint Positions - Right Leg')
    ax4.legend(fontsize=8, ncol=2)
    ax4.grid(True, alpha=0.3)
    
    # 5. Joint velocities (selected joints)
    ax5 = plt.subplot(num_rows, 2, 5)
    # Plot knee and ankle velocities for both legs
    ax5.plot(time_s, df['joint_vel_3'], label='L_knee', linewidth=1.0)
    ax5.plot(time_s, df['joint_vel_4'], label='L_ankle_pitch', linewidth=1.0)
    ax5.plot(time_s, df['joint_vel_9'], label='R_knee', linewidth=1.0)
    ax5.plot(time_s, df['joint_vel_10'], label='R_ankle_pitch', linewidth=1.0)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Velocity (rad/s)')
    ax5.set_title('Joint Velocities (Selected)')
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)
    
    # 6. Commands
    ax6 = plt.subplot(num_rows, 2, 6)
    ax6.plot(time_s, df['cmd_vx'], label='cmd_vx', linewidth=2.0)
    ax6.plot(time_s, df['cmd_vy'], label='cmd_vy', linewidth=2.0)
    ax6.plot(time_s, df['cmd_wz'], label='cmd_wz', linewidth=2.0)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Command')
    ax6.set_title('Velocity Commands')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    # 7. Actions (policy output)
    ax7 = plt.subplot(num_rows, 2, 7)
    # Plot a few representative actions
    ax7.plot(time_s, df['action_0'], label='L_hip_roll', linewidth=1.0, alpha=0.7)
    ax7.plot(time_s, df['action_2'], label='L_hip_pitch', linewidth=1.0, alpha=0.7)
    ax7.plot(time_s, df['action_3'], label='L_knee', linewidth=1.0, alpha=0.7)
    ax7.plot(time_s, df['action_6'], label='R_hip_roll', linewidth=1.0, alpha=0.7)
    ax7.plot(time_s, df['action_8'], label='R_hip_pitch', linewidth=1.0, alpha=0.7)
    ax7.plot(time_s, df['action_9'], label='R_knee', linewidth=1.0, alpha=0.7)
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Action')
    ax7.set_title('Policy Actions (Selected)')
    ax7.legend(fontsize=8, ncol=2)
    ax7.grid(True, alpha=0.3)
    
    # 8. Observation features (first few dimensions)
    ax8 = plt.subplot(num_rows, 2, 8)
    ax8.plot(time_s, df['obs_0'], label='obs_0 (ang_vel_x*0.25)', linewidth=1.0)
    ax8.plot(time_s, df['obs_1'], label='obs_1 (ang_vel_y*0.25)', linewidth=1.0)
    ax8.plot(time_s, df['obs_2'], label='obs_2 (ang_vel_z*0.25)', linewidth=1.0)
    ax8.plot(time_s, df['obs_3'], label='obs_3 (proj_grav_x)', linewidth=1.0)
    ax8.plot(time_s, df['obs_4'], label='obs_4 (proj_grav_y)', linewidth=1.0)
    ax8.plot(time_s, df['obs_5'], label='obs_5 (proj_grav_z)', linewidth=1.0)
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Observation Value')
    ax8.set_title('Processed Observations (First 6 dims)')
    ax8.legend(fontsize=7, ncol=2)
    ax8.grid(True, alpha=0.3)
    
    # 9 & 10. Torques (if available)
    if has_torque:
        # 9. Torques (left leg)
        ax9 = plt.subplot(num_rows, 2, 9)
        for i in range(6):
            ax9.plot(time_s, df[f'torque_{i}'], label=joint_names_left[i], linewidth=1.0)
        ax9.set_xlabel('Time (s)')
        ax9.set_ylabel('Torque (N·m)')
        ax9.set_title('Joint Torques - Left Leg')
        ax9.legend(fontsize=8, ncol=2)
        ax9.grid(True, alpha=0.3)
        
        # 10. Torques (right leg)
        ax10 = plt.subplot(num_rows, 2, 10)
        for i in range(6, 12):
            ax10.plot(time_s, df[f'torque_{i}'], label=joint_names_right[i-6], linewidth=1.0)
        ax10.set_xlabel('Time (s)')
        ax10.set_ylabel('Torque (N·m)')
        ax10.set_title('Joint Torques - Right Leg')
        ax10.legend(fontsize=8, ncol=2)
        ax10.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save figure
    output_file = csv_file.replace('.csv', '_visualization.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved visualization to: {output_file}")
    
    # Show interactive plot
    plt.show()

if __name__ == "__main__":
    main()

