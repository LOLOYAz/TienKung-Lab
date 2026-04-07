# Copyright (c) 2025-2026, The TienKung-Lab Project Developers.
# All rights reserved.
# Modifications are licensed under the BSD-3-Clause license.
#
# Convert LaFAN1 retargeted CSV data (lvhaidong/LAFAN1_Retargeting_Dataset)
# into TienKung-Lab motion_visualization format for G1 23DoF.
#
# LaFAN1 CSV G1 column order (29DoF):
#   root_joint(XYZQXQYQZQW)  [7 = 3pos + 4quat_xyzw]
#   left_hip_pitch, left_hip_roll, left_hip_yaw,
#   left_knee, left_ankle_pitch, left_ankle_roll,          -- left leg (6)
#   right_hip_pitch, right_hip_roll, right_hip_yaw,
#   right_knee, right_ankle_pitch, right_ankle_roll,       -- right leg (6)
#   waist_yaw, waist_roll, waist_pitch,                     -- waist (3)
#   left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw,
#   left_elbow, left_wrist_roll, left_wrist_pitch, left_wrist_yaw,  -- left arm (7)
#   right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw,
#   right_elbow, right_wrist_roll, right_wrist_pitch, right_wrist_yaw  -- right arm (7)
#
# Output format (motion_visualization):
#   root_pos(3), root_euler_XYZ(3),
#   left_leg_dof(6), right_leg_dof(6), left_arm_dof(5), right_arm_dof(5), waist(1),
#   root_lin_vel(3), root_ang_vel(3),
#   left_leg_vel(6), right_leg_vel(6), left_arm_vel(5), right_arm_vel(5), waist_vel(1)
# Total per frame: 58 values

import argparse
import numpy as np
from scipy.spatial.transform import Rotation


def convert_lafan1_csv_to_motion_vis(input_csv, output_txt, fps=30.0):
    """Convert LaFAN1 retargeted G1 CSV to TienKung-Lab motion_visualization format."""
    dt = 1.0 / fps

    # Load CSV (no header)
    raw = np.loadtxt(input_csv, delimiter=",")
    num_frames = raw.shape[0]
    print(f"Loaded {num_frames} frames from {input_csv}, shape={raw.shape}")

    # Parse columns
    root_pos = raw[:, 0:3]         # x, y, z
    root_quat_xyzw = raw[:, 3:7]   # qx, qy, qz, qw

    # Joint positions (29DoF from CSV)
    all_dof_pos = raw[:, 7:]  # 29 columns
    assert all_dof_pos.shape[1] >= 29, f"Expected at least 29 DoF columns, got {all_dof_pos.shape[1]}"

    # Extract G1 23DoF subset (drop waist_roll, waist_pitch, wrist_pitch, wrist_yaw)
    left_leg_pos = all_dof_pos[:, 0:6]    # 6
    right_leg_pos = all_dof_pos[:, 6:12]   # 6
    waist_yaw_pos = all_dof_pos[:, 12:13]  # 1
    left_arm_pos = all_dof_pos[:, 15:20]   # 5
    right_arm_pos = all_dof_pos[:, 22:27]  # 5

    euler_angles = Rotation.from_quat(root_quat_xyzw).as_euler("XYZ", degrees=False)
    euler_angles = np.unwrap(euler_angles, axis=0)

    # Compute velocities via finite differences
    root_lin_vel = (root_pos[1:] - root_pos[:-1]) / dt

    # Rotational velocity via scipy
    rot = Rotation.from_quat(root_quat_xyzw)
    q1_inv = rot[:-1].inv()
    dq = q1_inv * rot[1:]
    root_ang_vel = dq.as_rotvec() / dt

    left_leg_vel = (left_leg_pos[1:] - left_leg_pos[:-1]) / dt
    right_leg_vel = (right_leg_pos[1:] - right_leg_pos[:-1]) / dt
    left_arm_vel = (left_arm_pos[1:] - left_arm_pos[:-1]) / dt
    right_arm_vel = (right_arm_pos[1:] - right_arm_pos[:-1]) / dt
    waist_yaw_vel = (waist_yaw_pos[1:] - waist_yaw_pos[:-1]) / dt

    # Use frames [:-1] (one less due to velocity computation)
    n = num_frames - 1

    data_output = np.concatenate(
        (
            root_pos[:n],           # 3
            euler_angles[:n],       # 3
            left_leg_pos[:n],       # 6
            right_leg_pos[:n],      # 6
            left_arm_pos[:n],       # 5
            right_arm_pos[:n],      # 5
            waist_yaw_pos[:n],      # 1
            root_lin_vel,           # 3
            root_ang_vel,           # 3
            left_leg_vel,           # 6
            right_leg_vel,          # 6
            left_arm_vel,           # 5
            right_arm_vel,          # 5
            waist_yaw_vel,          # 1
        ),
        axis=1,
    )
    # Total: 3+3+6+6+5+5+1+3+3+6+6+5+5+1 = 58

    np.savetxt(output_txt, data_output, fmt="%f", delimiter=", ")
    with open(output_txt, "r") as f:
        frames_data = f.readlines()

    frames_data_len = len(frames_data)
    with open(output_txt, "w") as f:
        f.write("{\n")
        f.write('"LoopMode": "Wrap",\n')
        f.write(f'"FrameDuration": {1.0 / fps:.3f},\n')
        f.write('"EnableCycleOffsetPosition": true,\n')
        f.write('"EnableCycleOffsetRotation": true,\n')
        f.write('"MotionWeight": 0.5,\n\n')
        f.write('"Frames":\n[\n')

        for i, line in enumerate(frames_data):
            line_start_str = "  ["
            if i == frames_data_len - 1:
                f.write(line_start_str + line.rstrip() + "]\n")
            else:
                f.write(line_start_str + line.rstrip() + "],\n")

        f.write("]\n}")
    print(f"✅ Successfully converted {input_csv} → {output_txt} ({n} frames, {data_output.shape[1]} dims)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert LaFAN1 G1 CSV to TienKung-Lab motion_visualization format")
    parser.add_argument("--input_csv", type=str, required=True, help="Path to LaFAN1 G1 CSV file")
    parser.add_argument("--output_txt", type=str, required=True, help="Output TXT path")
    parser.add_argument("--fps", type=float, default=30.0, help="Frame rate (LaFAN1 G1 data is 30 FPS)")
    args = parser.parse_args()

    convert_lafan1_csv_to_motion_vis(args.input_csv, args.output_txt, args.fps)
