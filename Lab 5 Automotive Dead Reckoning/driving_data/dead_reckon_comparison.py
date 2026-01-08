import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate, signal

# Reading driving data from csv
df = pd.read_csv('driving_data_0_imu.csv')

acc_x = df['acc_x'].values
acc_y = df['acc_y'].values
gyro_z = df['gyro_z'].values
timestamps = df['t'].values
timestamps = timestamps - timestamps[0]

# Getting corrected forward velocity using gps and IMU fusion
stationary_mask = timestamps < 10
acc_x_bias = np.mean(acc_x[stationary_mask])
acc_x_corrected = acc_x - acc_x_bias

vel_imu_raw = integrate.cumulative_trapezoid(acc_x_corrected, timestamps, initial=0)

# Calculating GPS velocity, first reading csv before variable assigning
gps_df = pd.read_csv('driving_data_0_gps.csv')

lat = np.radians(gps_df['latitude'].values)
lon = np.radians(gps_df['longitude'].values)
gps_time = gps_df['t'].values
gps_time = gps_time - gps_time[0]

R = 6371000
vel_gps = np.zeros(len(lat))

# Haversine formula to compute shortest distance between consecutive GPS points
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

# Fusing GPS and IMU velocity (using optimal cutoff = 0.10 Hz)
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

# Calculating Omega*x(dot) using the corrected velocity
omega_X_dot = gyro_z * vel_fused

# Filtering observed lateral acceleration lightly
cutoff_acc = 1.0 
lpf_norm_acc = cutoff_acc / nyq
b_lpf_acc, a_lpf_acc = signal.butter(2, lpf_norm_acc, btype='low')
y_obs_filtered = signal.filtfilt(b_lpf_acc, a_lpf_acc, acc_y)

print(f"forward velocity (fused) range: [{vel_fused.min():.2f}, {vel_fused.max():.2f}] m/s")
print(f"Omega*X_dot range: [{omega_X_dot.min():.2f}, {omega_X_dot.max():.2f}] m/s²")
print(f"y_obs (filtered) range: [{y_obs_filtered.min():.2f}, {y_obs_filtered.max():.2f}] m/s²")

# Plotting comparison
plt.figure(figsize=(14, 6))

plt.plot(timestamps, omega_X_dot, 'b-', linewidth=1.5, label='ωẊ (modeled)')
plt.plot(timestamps, y_obs_filtered, 'r-', linewidth=1.5, label='filtered ÿ_obs (observed)')

plt.xlabel('time (s)')
plt.ylabel('acceleration (m/s²)')
plt.title('dead reckoning using IMU and GPS data')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

plt.savefig('dead_reckoning_comparison.png', dpi=300, bbox_inches='tight')

print("\nplot saved")
