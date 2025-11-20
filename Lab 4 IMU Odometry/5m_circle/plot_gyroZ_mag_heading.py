#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = Path("/home/kiran-sairam/imu_ws/analysis/5m_circle/5m_circle_0_imu.csv")
out_fig = csv_path.with_name("Gyro Z rate + angle + mag heading fig4.png")

bias_samples = 80  # first ~2 s at ~40 Hz


def main():
    df = pd.read_csv(csv_path)

    # time (use bag time, start at zero)
    if "bag_t_sec" in df.columns:
        t = df["bag_t_sec"].to_numpy(float)
    else:
        t = df["ros_header_t_sec"].to_numpy(float)

    # drop repeated timestamps
    keep = np.r_[True, np.diff(t) > 0]
    df = df.loc[keep].reset_index(drop=True)
    if "bag_t_sec" in df.columns:
        t = df["bag_t_sec"].to_numpy(float)
    else:
        t = df["ros_header_t_sec"].to_numpy(float)
    t = t - t[0]

    # gyro z rate and bias
    gz = df["gyro_z"].to_numpy(float)
    n0 = min(bias_samples, len(gz))
    gz_bias = float(np.mean(gz[:n0]))
    gz_d = gz - gz_bias

    # integrate gyro z once to get angle (radians)
    dt = np.diff(t, prepend=t[0])
    angle_z = np.cumsum(0.5 * (gz_d + np.r_[gz_d[:1], gz_d[:-1]]) * dt)

    # magnetometer heading (enu: atan2(y, x))
    mx = df["mag_x"].to_numpy(float)
    my = df["mag_y"].to_numpy(float)
    heading = np.arctan2(my, mx)         # [-pi, pi]
    heading_unwrapped = np.unwrap(heading)

    fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    axs[0].plot(t, gz, lw=1.0, label="gyro_z (raw)")
    axs[0].plot(t, gz_d, lw=1.0, alpha=0.9, label="gyro_z (bias removed)")
    axs[0].set_ylabel("rate (rad/s)")
    axs[0].set_title("z rate, integrated z angle, and magnetometer heading")
    axs[0].grid(True)
    axs[0].legend(loc="upper right")

    axs[1].plot(t, angle_z, lw=1.5, label="angle_z = ∫ (gyro_z − bias) dt")
    axs[1].set_ylabel("angle (rad)")
    axs[1].grid(True)
    axs[1].legend(loc="upper left")

    axs[2].plot(t, heading_unwrapped, lw=1.2, label="mag heading = unwrap(atan2(y, x))")
    axs[2].set_xlabel("time (s)")
    axs[2].set_ylabel("heading (rad)")
    axs[2].grid(True)
    axs[2].legend(loc="upper left")

    fig.tight_layout()
    fig.savefig(out_fig, dpi=300)
    plt.close(fig)

    print(f"saved {out_fig}")
    print(f"gyro_z bias ≈ {gz_bias:.6f} rad/s")
    print(
        f"mag heading span ≈ {np.degrees(heading).min():.1f}° "
        f"→ {np.degrees(heading).max():.1f}°"
    )


if __name__ == "__main__":
    main()
