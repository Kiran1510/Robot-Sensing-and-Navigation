import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate, signal

# Reading driving data from csv
df = pd.read_csv('driving_data_0_imu.csv')

acc_x = df['acc_x'].values
timestamps = df['t'].values
timestamps = timestamps - timestamps[0]

# Correcting bias from stationary period
stationary_mask = timestamps < 10
acc_x_bias = np.mean(acc_x[stationary_mask])
acc_x_corrected = acc_x - acc_x_bias

print(f"acceleration bias: {acc_x_bias:.4f} m/s^2")

# Integrating to get velocity
vel_imu_raw = integrate.cumulative_trapezoid(acc_x_corrected, timestamps, initial=0)

# Calculating GPS velocity
gps_df = pd.read_csv('driving_data_0_gps.csv')

lat = np.radians(gps_df['latitude'].values)
lon = np.radians(gps_df['longitude'].values)
gps_time = gps_df['t'].values
gps_time = gps_time - gps_time[0]

# Using the Haversine formula to calculate distance between GPS points, and thus, velocity
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

# Interpolating GPS velocity to IMU timestamps
vel_gps_interp = np.interp(timestamps, gps_time, vel_gps)

# Complementary filter with ideal cutoff value of 0.10 Hz
fs = 1 / np.mean(np.diff(timestamps))
cutoff = 0.10 
nyq = 0.5 * fs

# Low pass filter GPS
lpf_norm = cutoff / nyq
b_lpf, a_lpf = signal.butter(2, lpf_norm, btype='low')
vel_gps_lpf = signal.filtfilt(b_lpf, a_lpf, vel_gps_interp)

# High pass filter IMU
hpf_norm = cutoff / nyq
b_hpf, a_hpf = signal.butter(2, hpf_norm, btype='high')
vel_imu_hpf = signal.filtfilt(b_hpf, a_hpf, vel_imu_raw)

# Fusing velocity
vel_fused = vel_gps_lpf + vel_imu_hpf
vel_fused[vel_fused < 0] = 0

# Keeping HPF only approach for comparison
vel_imu_hpf_only = signal.filtfilt(b_hpf, a_hpf, vel_imu_raw)
vel_imu_hpf_only[vel_imu_hpf_only < 0] = 0

print(f"\nGPS velocity range: [{vel_gps.min():.2f}, {vel_gps.max():.2f}] m/s")
print(f"IMU velocity (raw): [{vel_imu_raw.min():.2f}, {vel_imu_raw.max():.2f}] m/s")
print(f"IMU velocity (HPF only): [{vel_imu_hpf_only.min():.2f}, {vel_imu_hpf_only.max():.2f}] m/s")
print(f"fused velocity: [{vel_fused.min():.2f}, {vel_fused.max():.2f}] m/s")

# Plotting comparison
fig, axes = plt.subplots(3, 1, figsize=(14, 12))

# Before adjusting
axes[0].plot(timestamps, vel_imu_raw, 'b-', linewidth=1, alpha=0.7, label='IMU velocity (raw, drifts)')
axes[0].plot(timestamps, vel_gps_interp, 'r-', linewidth=1, label='GPS velocity')
axes[0].set_ylabel('velocity (m/s)')
axes[0].set_title('before adjustment')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# HPF only (0.10 Hz)
axes[1].plot(timestamps, vel_imu_hpf_only, 'b-', linewidth=1.5, label='IMU velocity (HPF 0.10 Hz)')
axes[1].plot(timestamps, vel_gps_interp, 'r-', linewidth=1, alpha=0.7, label='GPS velocity')
axes[1].set_ylabel('velocity (m/s)')
axes[1].set_title('IMU with HPF (0.10 Hz)')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

# Complementary filter (0.10 Hz)
axes[2].plot(timestamps, vel_fused, 'g-', linewidth=2, label='fused velocity (complementary, 0.10 Hz)')
axes[2].plot(timestamps, vel_gps_interp, 'r-', linewidth=1, alpha=0.7, label='GPS velocity')
axes[2].set_xlabel('time (s)')
axes[2].set_ylabel('velocity (m/s)')
axes[2].set_title('complementary filter (GPS + IMU, cutoff = 0.10 Hz)')
axes[2].legend()
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('fig_4.png', dpi=300, bbox_inches='tight')

print("\nplot saved as 'fig_4.png'")
