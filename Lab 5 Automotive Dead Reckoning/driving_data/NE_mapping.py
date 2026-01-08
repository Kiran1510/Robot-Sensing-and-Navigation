import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate, signal

# Reading driving data from csv
df = pd.read_csv('driving_data_0_imu.csv')

acc_x = df['acc_x'].values
mag_x = df['mag_x'].values
mag_y = df['mag_y'].values
gyro_z = df['gyro_z'].values
timestamps = df['t'].values
timestamps = timestamps - timestamps[0]

# Getting fused velocity
stationary_mask = timestamps < 10
acc_x_bias = np.mean(acc_x[stationary_mask])
acc_x_corrected = acc_x - acc_x_bias

vel_imu_raw = integrate.cumulative_trapezoid(acc_x_corrected, timestamps, initial=0)

# Calculating GPS velocity
gps_df = pd.read_csv('driving_data_0_gps.csv')

lat = np.radians(gps_df['latitude'].values)
lon = np.radians(gps_df['longitude'].values)
gps_time = gps_df['t'].values
gps_time = gps_time - gps_time[0]

# Getting UTM coordinates from csv table
utm_easting = gps_df['utm_easting'].values
utm_northing = gps_df['utm_northing'].values

# calculating GPS velocity using haversine formula
R = 6371000
vel_gps = np.zeros(len(lat))

for i in range(1, len(lat)):
    dlat = lat[i] - lat[i-1]
    dlon = lon[i] - lon[i-1]
    
    a = np.sin(dlat/2)**2 + np.cos(lat[i-1]) * np.cos(lat[i]) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    distance = R * c
    
    dt = gps_time[i] - gps_time[i-1]
    if dt > 0:
        vel_gps[i] = distance / dt

vel_gps_interp = np.interp(timestamps, gps_time, vel_gps)

# Fusing GPS and IMU velocity using filter
fs = 1 / np.mean(np.diff(timestamps))
nyq = 0.5 * fs
cutoff_vel = 0.10

lpf_norm = cutoff_vel / nyq
b_lpf, a_lpf = signal.butter(2, lpf_norm, btype='low')
vel_gps_lpf = signal.filtfilt(b_lpf, a_lpf, vel_gps_interp)

hpf_norm = cutoff_vel / nyq
b_hpf, a_hpf = signal.butter(2, hpf_norm, btype='high')
vel_imu_hpf = signal.filtfilt(b_hpf, a_hpf, vel_imu_raw)

vel_fused = vel_gps_lpf + vel_imu_hpf
vel_fused[vel_fused < 0] = 0

# Getting fused yaw
hard_iron_offset = np.array([0.00001978, 0.00001289])
soft_iron_matrix = np.array([[1.00017403, -0.00836799],
                              [-0.00836799, 0.99996603]])

mag_raw = np.column_stack([mag_x, mag_y])
mag_centered = mag_raw - hard_iron_offset
mag_calibrated = (soft_iron_matrix @ mag_centered.T).T

yaw_mag = np.arctan2(mag_calibrated[:, 1], mag_calibrated[:, 0])
yaw_mag_unwrapped = np.unwrap(yaw_mag)

gyro_bias = np.mean(gyro_z)
gyro_z_corrected = gyro_z - gyro_bias

yaw_gyro = integrate.cumulative_trapezoid(gyro_z_corrected, timestamps, initial=0)
yaw_gyro = yaw_gyro - yaw_gyro[0] + yaw_mag_unwrapped[0]

cutoff_yaw = 0.1
lpf_norm_yaw = cutoff_yaw / nyq
b_lpf_yaw, a_lpf_yaw = signal.butter(2, lpf_norm_yaw, btype='low')
yaw_mag_lpf = signal.filtfilt(b_lpf_yaw, a_lpf_yaw, yaw_mag_unwrapped)

hpf_norm_yaw = cutoff_yaw / nyq
b_hpf_yaw, a_hpf_yaw = signal.butter(2, hpf_norm_yaw, btype='high')
yaw_gyro_hpf = signal.filtfilt(b_hpf_yaw, a_hpf_yaw, yaw_gyro)

yaw_fused = yaw_mag_lpf + yaw_gyro_hpf

# Decomposing velocity into easting and northing components
ve = vel_fused * np.cos(yaw_fused)
vn = vel_fused * np.sin(yaw_fused)

# Integrating to get displacement
xe = integrate.cumulative_trapezoid(ve, timestamps, initial=0)
xn = integrate.cumulative_trapezoid(vn, timestamps, initial=0)

# Zero GPS trajectory to start at origin
easting_zeroed = utm_easting - utm_easting[0]
northing_zeroed = utm_northing - utm_northing[0]

# Calculating GPS heading for alignment
gps_heading_initial = np.arctan2(utm_northing[1] - utm_northing[0], utm_easting[1] - utm_easting[0])
imu_heading_initial = yaw_fused[0]

# Rotating angle to align IMU with GPS
rotation_angle = gps_heading_initial - imu_heading_initial

# Rotating IMU trajectory to align with GPS
xe_rotated = xe * np.cos(rotation_angle) - xn * np.sin(rotation_angle)
xn_rotated = xe * np.sin(rotation_angle) + xn * np.cos(rotation_angle)

print(f"GPS heading initial: {np.degrees(gps_heading_initial):.2f} degrees")
print(f"IMU heading initial: {np.degrees(imu_heading_initial):.2f} degrees")
print(f"rotation applied: {np.degrees(rotation_angle):.2f} degrees")

# Plotting trajectory comparison
plt.figure(figsize=(12, 10))

plt.plot(xe_rotated, xn_rotated, 'b-', linewidth=2, label='IMU trajectory')
plt.plot(easting_zeroed, northing_zeroed, 'r-', linewidth=2, label='GPS trajectory')

plt.xlabel('easting position (m)')
plt.ylabel('northing position (m)')
plt.title('estimated vehicle trajectory')
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.tight_layout()

plt.savefig('fig_6.png', dpi=300, bbox_inches='tight')

print("\nplot saved as 'fig_6.png'")
print(f"IMU trajectory range: E=[{xe_rotated.min():.1f}, {xe_rotated.max():.1f}], N=[{xn_rotated.min():.1f}, {xn_rotated.max():.1f}]")
print(f"GPS trajectory range: E=[{easting_zeroed.min():.1f}, {easting_zeroed.max():.1f}], N=[{northing_zeroed.min():.1f}, {northing_zeroed.max():.1f}]")
