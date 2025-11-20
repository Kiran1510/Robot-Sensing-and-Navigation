#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

#  CONFIG 
CSV_FILE = "11sec_motion_0.csv"        # Your CSV file name
OUT_PNG  = "gyro_rotation_deg.png"     # Output image
# --------------------------

# Load CSV
df = pd.read_csv(CSV_FILE)

# Time 
if "ros_header_t_sec" in df.columns:
    t = df["ros_header_t_sec"].to_numpy()
elif "bag_t_sec" in df.columns:
    t = df["bag_t_sec"].to_numpy()
else:
    t = np.arange(len(df)) / 40.0  # assume 40 Hz fallback

# Gyro (rad/s to deg/s)
rad2deg = 180.0 / np.pi
gx = df["gyro_x"].to_numpy() * rad2deg
gy = df["gyro_y"].to_numpy() * rad2deg
gz = df["gyro_z"].to_numpy() * rad2deg

# Plot
plt.figure(figsize=(10,6))
plt.plot(t, gx, 'r-', marker='o', markevery=30, label='Gyro X (deg/s)')
plt.plot(t, gy, 'g-', marker='x', markevery=30, label='Gyro Y (deg/s)')
plt.plot(t, gz, 'b-', marker='s', markevery=30, label='Gyro Z (deg/s)')

plt.title("Rotational Rate from Gyroscope (deg/s)")
plt.xlabel("Time (s)")
plt.ylabel("Angular Rate (deg/s)")
plt.grid(True, alpha=0.4)
plt.legend()
plt.tight_layout()

# Save to folder
out_path = Path(OUT_PNG)
plt.savefig(out_path, dpi=300)
print("Saved plot: {out_path}")

plt.close()
