#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# Path to CSV file
csv_path = Path("/home/kiran-sairam/imu_ws/analysis/11sec_motion/11sec_motion_0.csv")
out_png = Path("/home/kiran-sairam/imu_ws/analysis/11sec_motion/acceleration_xyz.png")

print(f"Loading CSV: {csv_path}")
df = pd.read_csv(csv_path)
print(f"Loaded {len(df)} rows")

# Extract time
if "ros_header_t_sec" in df.columns:
    t = df["ros_header_t_sec"].to_numpy()
    t = t - t[0]  # start time at zero
else:
    # 40Hz sampling
    print("No ROS time column — assuming 40 Hz sample rate.")
    dt = 1.0 / 40.0
    t = np.arange(len(df)) * dt

# accelerometer data
ax = df["acc_x"].to_numpy()
ay = df["acc_y"].to_numpy()
az = df["acc_z"].to_numpy()

# Plot
plt.figure(figsize=(10,6))
plt.plot(t, ax, marker='o', markersize=2, linestyle='-', label='Accel X')
plt.plot(t, ay, marker='s', markersize=2, linestyle='--', label='Accel Y')
plt.plot(t, az, marker='^', markersize=2, linestyle='-.', label='Accel Z')

plt.title("Acceleration vs Time (m/s²)")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s²)")
plt.grid(True, alpha=0.4)
plt.legend()
plt.tight_layout()

# Save
out_png.parent.mkdir(parents=True, exist_ok=True)
plt.savefig(out_png, dpi=200)
plt.close()

print("Saved plot to: {out_png}")
