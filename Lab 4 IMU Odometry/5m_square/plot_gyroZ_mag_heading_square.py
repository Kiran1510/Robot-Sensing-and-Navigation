#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = Path("/home/kiran-sairam/imu_ws/analysis/5m_square/5m_square_0_imu.csv")
out_fig = csv_path.with_name("Gyro Z and mag heading fig13.png")

bias_secs = 2.0
smooth_win = 5
dpi = 300


def moving_average(x, w):
    if w <= 1:
        return x
    k = np.ones(int(w)) / int(w)
    return np.convolve(x, k, mode="same")


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

    return t - t[0], df


def main():
    df_all = pd.read_csv(csv_path)
    t, df = load_time(df_all)

    gz = df["gyro_z"].to_numpy(float)
    mx = df["mag_x"].to_numpy(float)
    my = df["mag_y"].to_numpy(float)

    n0 = max(1, np.searchsorted(t, bias_secs, side="right"))
    bias_z = float(np.mean(gz[:n0]))
    gz_d = gz - bias_z

    angle_z = integrate_trap(gz_d, t)
    hdg_mag = np.unwrap(np.arctan2(my, mx))

    angle_z -= angle_z[0]
    hdg_mag -= hdg_mag[0]

    gz_plot = moving_average(gz, smooth_win)
    gzd_plot = moving_average(gz_d, smooth_win)

    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(12, 9), sharex=True,
        gridspec_kw=dict(height_ratios=[1, 1, 1])
    )

    ax1.plot(t, gz_plot, label="gyro_z (raw)")
    ax1.plot(t, gzd_plot, label="gyro_z (bias removed)")
    ax1.set_ylabel("rate (rad/s)")
    ax1.set_title("z rate, integrated z angle, and magnetometer heading")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    ax2.plot(t, angle_z, label="angle_z")
    ax2.set_ylabel("angle (rad)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper left")

    ax3.plot(t, hdg_mag, label="mag heading")
    ax3.set_ylabel("heading (rad)")
    ax3.set_xlabel("time (s)")
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper left")

    plt.tight_layout()
    plt.savefig(out_fig, dpi=dpi)
    plt.close()


if __name__ == "__main__":
    main()
