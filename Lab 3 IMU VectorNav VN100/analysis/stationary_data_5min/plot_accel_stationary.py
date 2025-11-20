#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

csv_path = Path("/home/kiran-sairam/imu_ws/analysis/stationary_data_5min/stationary_data_5min_0.csv")
outdir = Path("/home/kiran-sairam/imu_ws/analysis/stationary_data_5min")

print(f"Loading CSV: {csv_path}")
df = pd.read_csv(csv_path)
print(f"Loaded {len(df)} rows")

if "ros_header_t_sec" in df.columns:
    t = df["ros_header_t_sec"].to_numpy() - df["ros_header_t_sec"].iloc[0]
else:
    dt = 1.0 / 40.0
    t = np.arange(len(df)) * dt

acc_cols = ["acc_x", "acc_y", "acc_z"]
titles = ["Accel X (m/s²)", "Accel Y (m/s²)", "Accel Z (m/s²)"]

fig, axes = plt.subplots(1, 3, figsize=(15, 4))
for i, col in enumerate(acc_cols):
    axes[i].plot(t, df[col], marker='.', markersize=2, linewidth=1)
    axes[i].set_title(titles[i])
    axes[i].set_xlabel("Time (s)")
    axes[i].set_ylabel("Acceleration (m/s²)")
    axes[i].grid(alpha=0.3)

plt.suptitle("Stationary IMU Accelerometer Data (X, Y, Z)")
plt.tight_layout()
outfile = outdir / "stationary_accel_xyz.png"
plt.savefig(outfile, dpi=200)
plt.close()
print("Saved {outfile}")
