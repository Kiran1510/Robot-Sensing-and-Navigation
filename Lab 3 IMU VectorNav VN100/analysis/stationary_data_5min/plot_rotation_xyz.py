#!/usr/bin/env python3
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# python3 plot_rotation_xyz.py /path/to/file.csv
csv_path = sys.argv[1]
df = pd.read_csv(csv_path)

# Time 40 Hz
if 'bag_t' in df.columns:
    t = df['bag_t'].to_numpy(dtype=float)
elif 'ros_t' in df.columns:
    t = df['ros_t'].to_numpy(dtype=float)
else:
    t = np.arange(len(df), dtype=float) * (1.0/40.0)

# Parse Yaw, Pitch, Roll (deg) from $VNYMR line in 'raw' column
# Format: $VNYMR, Yaw,Pitch,Roll, MagX,MagY,MagZ, AccX,AccY,AccZ, GyroX,GyroY,GyroZ*CS
parts = df['raw'].str.split(',', expand=True)
yaw   = parts[1].astype(float).to_numpy()
pitch = parts[2].astype(float).to_numpy()
roll  = parts[3].astype(float).to_numpy()

# Plot 1x3 matrix (Roll, Pitch, Yaw in degrees)
fig, axes = plt.subplots(1, 3, figsize=(15, 4))
axes[0].plot(t, roll,  linewidth=1.0)
axes[1].plot(t, pitch, linewidth=1.0)
axes[2].plot(t, yaw,   linewidth=1.0)

axes[0].set_title('Roll (deg)')
axes[1].set_title('Pitch (deg)')
axes[2].set_title('Yaw (deg)')

for ax in axes:
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Degrees')
    ax.grid(True, alpha=0.3)

plt.tight_layout()
out = 'rotation_xyz.png'
plt.savefig(out, dpi=200)
print('Saved:', out)
