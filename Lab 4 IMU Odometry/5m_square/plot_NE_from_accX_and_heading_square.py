#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = Path("/home/kiran-sairam/imu_ws/analysis/5m_square/5m_square_0_imu.csv")
out_png = csv_path.with_name("NE_position_from_accX_and_heading_square.png")

acc_bias_secs = 2.0   # seconds at start to estimate accel bias
gyro_bias_secs = 2.0  # seconds at start to estimate gyro bias
dpi = 300


def integrate_trap(y, t):
    dt = np.diff(t, prepend=t[0])
    return np.cumsum(0.5 * (y + np.r_[y[:1], y[:-1]]) * dt)


def load_time(df):
    if "bag_t_sec" in df.columns:
        t = df["bag_t_sec"].to_numpy(float)
    else:
        t = df["ros_header_t_sec"].to_numpy(float)

    keep = np.r_[True, np.diff(t) > 0]
    df = df.loc[keep].reset_index(drop=True)

    if "bag_t_sec" in df.columns:
        t = df["bag_t_sec"].to_numpy(float)
    else:
        t = df["ros_header_t_sec"].to_numpy(float)

    t = t - t[0]
    return t, df


def main():
    df0 = pd.read_csv(csv_path)
    t, df = load_time(df0)
    dt = np.diff(t, prepend=t[0])

    acc_x = df["acc_x"].to_numpy(float)
    gyro_z = df["gyro_z"].to_numpy(float)
    mx = df["mag_x"].to_numpy(float)
    my = df["mag_y"].to_numpy(float)

    # accel bias and de-trended forward velocity
    n_acc0 = max(1, np.searchsorted(t, acc_bias_secs, side="right"))
    acc_bias = float(np.mean(acc_x[:n_acc0]))
    acc_x_d = acc_x - acc_bias
    v_fwd = integrate_trap(acc_x_d, t)

    coeffs = np.polyfit(t, v_fwd, 1)
    trend = np.polyval(coeffs, t)
    v_fwd = v_fwd - trend

    # gyro bias and raw integrated heading
    n_gyr0 = max(1, np.searchsorted(t, gyro_bias_secs, side="right"))
    gyro_bias = float(np.mean(gyro_z[:n_gyr0]))
    gyro_z_d = gyro_z - gyro_bias
    hdg_gyro = integrate_trap(gyro_z_d, t)
    hdg_gyro -= hdg_gyro[0]

    # magnetometer heading
    hdg_mag = np.unwrap(np.arctan2(my, mx))
    hdg_mag -= hdg_mag[0]

    # fit gyro heading to magnetometer heading
    A = np.vstack([hdg_gyro, np.ones_like(hdg_gyro)]).T
    a, b = np.linalg.lstsq(A, hdg_mag, rcond=None)[0]
    hdg_gyro_cal = a * hdg_gyro + b

    # project velocity into N/E and integrate to position
    dE_mag = v_fwd * dt * np.sin(hdg_mag)
    dN_mag = v_fwd * dt * np.cos(hdg_mag)
    dE_gyr = v_fwd * dt * np.sin(hdg_gyro_cal)
    dN_gyr = v_fwd * dt * np.cos(hdg_gyro_cal)

    E_mag = np.cumsum(dE_mag) - dE_mag[0]
    N_mag = np.cumsum(dN_mag) - dN_mag[0]
    E_gyr = np.cumsum(dE_gyr) - dE_gyr[0]
    N_gyr = np.cumsum(dN_gyr) - dN_gyr[0]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(E_mag, N_mag, label="mag heading")
    ax.plot(E_gyr, N_gyr, label="gyro heading")

    ax.plot(E_mag[0], N_mag[0], "bo", label="start (mag)")
    ax.plot(E_mag[-1], N_mag[-1], "bx", label="end (mag)")
    ax.plot(E_gyr[0], N_gyr[0], "ro", label="start (gyro)")
    ax.plot(E_gyr[-1], N_gyr[-1], "rx", label="end (gyro)")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("Square walking: estimated position (N vs E)")
    ax.legend(loc="upper left")

    plt.tight_layout()
    plt.savefig(out_png, dpi=dpi)
    plt.close()


if __name__ == "__main__":
    main()
