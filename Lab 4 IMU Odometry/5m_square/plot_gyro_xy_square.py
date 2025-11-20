#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = Path("/home/kiran-sairam/imu_ws/analysis/5m_square/5m_square_0_imu.csv")

bias_secs = 2.0      # seconds used to estimate initial bias
smooth_win = 5       # moving-average window for plots; <=1 disables
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


def make_fig(axis_name, rate, t, out_png):
    n0 = max(1, np.searchsorted(t, bias_secs, side="right"))
    bias = float(np.mean(rate[:n0]))
    rate_d = rate - bias

    rate_plot = moving_average(rate, smooth_win)
    rate_d_plot = moving_average(rate_d, smooth_win)
    angle = integrate_trap(rate_d, t)

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True,
        gridspec_kw=dict(height_ratios=[1, 1])
    )

    ax1.plot(t, rate_plot, label=f"gyro_{axis_name} (raw)")
    ax1.plot(t, rate_d_plot, label=f"gyro_{axis_name} (bias removed)")
    ax1.set_ylabel("rate (rad/s)")
    ax1.set_title(f"gyro {axis_name}: rate and integrated angle")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    ax2.plot(t, angle, label=f"angle_{axis_name}")
    ax2.set_ylabel("angle (rad)")
    ax2.set_xlabel("time (s)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper left")

    plt.tight_layout()
    plt.savefig(out_png, dpi=dpi)
    plt.close()


def main():
    df_all = pd.read_csv(csv_path)
    t, df = load_time(df_all)

    gx = df["gyro_x"].to_numpy(float)
    gy = df["gyro_y"].to_numpy(float)

    make_fig("x", gx, t, csv_path.with_name("Gyro X rate and angle fig10.png"))
    make_fig("y", gy, t, csv_path.with_name("Gyro Y rate and angle fig11.png"))


if __name__ == "__main__":
    main()
