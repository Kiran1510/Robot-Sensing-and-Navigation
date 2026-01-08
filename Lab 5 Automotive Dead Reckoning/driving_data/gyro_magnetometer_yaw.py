import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate

# Reading the driving data from csv
df = pd.read_csv('driving_data_0_imu.csv')

mag_x = df['mag_x'].values
mag_y = df['mag_y'].values
gyro_z = df['gyro_z'].values
timestamps = df['t'].values
timestamps = timestamps - timestamps[0]

# Correcting gyro bias
gyro_bias = np.mean(gyro_z)
gyro_z_corrected = gyro_z - gyro_bias

# Applying magnetometer calibration
hard_iron_offset = np.array([0.00001978, 0.00001289])
soft_iron_matrix = np.array([[1.00017403, -0.00836799],
                              [-0.00836799, 0.99996603]])

mag_raw = np.column_stack([mag_x, mag_y])
mag_centered = mag_raw - hard_iron_offset
mag_calibrated = (soft_iron_matrix @ mag_centered.T).T

# Calculating calibrated yaw from magnetometer and unwrapping (prevents discontinuities/artifacts in the plot)
yaw_calibrated = np.arctan2(mag_calibrated[:, 1], mag_calibrated[:, 0])
yaw_calibrated_unwrapped = np.unwrap(yaw_calibrated)

# Integrating gyro to get yaw
yaw_gyro = integrate.cumulative_trapezoid(gyro_z_corrected, timestamps, initial=0)

# Aligning the start points
yaw_gyro = yaw_gyro - yaw_gyro[0] + yaw_calibrated_unwrapped[0]

print(f"calibrated yaw range: {yaw_calibrated_unwrapped.min():.2f} to {yaw_calibrated_unwrapped.max():.2f} rad")
print(f"gyro yaw range: {yaw_gyro.min():.2f} to {yaw_gyro.max():.2f} rad")

# Plotting
plt.figure(figsize=(12, 6))
plt.plot(timestamps, yaw_calibrated_unwrapped, 'r-', linewidth=1, label='calibrated yaw')
plt.plot(timestamps, yaw_gyro, 'b-', linewidth=1, label='gyro yaw')
plt.xlabel('time (s)')
plt.ylabel('yaw (rad)')
plt.title('yaw from gyro and magnetometer')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('fig_2.png', dpi=300, bbox_inches='tight')

print("\nplot saved as fig_2.png")
