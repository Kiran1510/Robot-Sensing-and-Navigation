#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = Path("/home/kiran-sairam/imu_ws/analysis/5m_square/5m_square_0_imu.csv")
dpi = 300

acc_bias_secs = 2.0     # seconds at start used for accel bias
smooth_win = 5          # moving average window; <=1 disables


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


def make_fig(axis_name, acc, t, out_png):
    n0 = max(1, np.searchsorted(t, acc_bias_secs, side="right"))
    bias = float(np.mean(acc[:n0]))
    acc_d = acc - bias

    vel = integrate_trap(acc_d, t)

    acc_raw_plot = moving_average(acc, smooth_win)
    acc_d_plot = moving_average(acc_d, smooth_win)
    vel_plot = moving_average(vel, smooth_win)

    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(12, 9), sharex=True,
        gridspec_kw=dict(height_ratios=[1, 1, 1])
    )

    ax1.plot(t, acc_raw_plot, label=f"acc_{axis_name} (raw)")
    ax1.set_ylabel("acc (m/s²)")
    ax1.set_title(f"acceleration and velocity – axis {axis_name}")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    ax2.plot(t, acc_d_plot, label=f"acc_{axis_name} (bias removed)")
    ax2.set_ylabel("acc (m/s²)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")

    ax3.plot(t, vel_plot, label=f"vel_{axis_name} = ∫ acc dt")
    ax3.set_ylabel("vel (m/s)")
    ax3.set_xlabel("time (s)")
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper right")

    plt.tight_layout()
    plt.savefig(out_png, dpi=dpi)
    plt.close()


def main():
    df_all = pd.read_csv(csv_path)
    t, df = load_time(df_all)

    ax = df["acc_x"].to_numpy(float)
    ay = df["acc_y"].to_numpy(float)
    az = df["acc_z"].to_numpy(float)

    make_fig("x", ax, t, csv_path.with_name("Accel and velocity X fig13.png"))
    make_fig("y", ay, t, csv_path.with_name("Accel and velocity Y fig14.png"))
    make_fig("z", az, t, csv_path.with_name("Accel and velocity Z fig15.png"))


if __name__ == "__main__":
    main()
