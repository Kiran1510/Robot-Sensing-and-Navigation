#!/usr/bin/env python3
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# Input CSV
csv_path = "/home/kiran-sairam/imu_ws/analysis/11sec_motion/11sec_motion_0.csv"
df = pd.read_csv(csv_path)

# Extract time
if 'ros_header_t_sec' in df.columns:
    t = df['ros_header_t_sec'].to_numpy(dtype=float)
elif 'bag_t_sec' in df.columns:
    t = df['bag_t_sec'].to_numpy(dtype=float)
else:
    t = np.arange(len(df)) * (1.0/40.0)

# Normalize time to zero
t = t - t[0]

# Parse yaw, pitch, roll from raw VNYMR
parts = df['raw'].str.split(',', expand=True)
yaw   = parts[1].astype(float).to_numpy()
pitch = parts[2].astype(float).to_numpy()
roll  = parts[3].astype(float).to_numpy()

# Plot
fig, axes = plt.subplots(1, 3, figsize=(15, 4))

axes[0].plot(t, roll, linewidth=1.2)
axes[1].plot(t, pitch, linewidth=1.2)
axes[2].plot(t, yaw, linewidth=1.2)

axes[0].set_title('Roll (deg)')
axes[1].set_title('Pitch (deg)')
axes[2].set_title('Yaw (deg)')

for ax in axes:
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Degrees')
    ax.grid(alpha=0.3)

plt.tight_layout()

out_path = Path("/home/kiran-sairam/imu_ws/analysis/11sec_motion/rotation_xyz_11sec.png")
plt.savefig(out_path, dpi=200)
print("Saved:", out_path)
