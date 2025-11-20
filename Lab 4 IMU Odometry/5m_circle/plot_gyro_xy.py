#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = Path("/home/kiran-sairam/imu_ws/analysis/5m_circle/5m_circle_0_imu.csv")

out_x = csv_path.with_name("Gyro X rate + angle fig2.png")
out_y = csv_path.with_name("Gyro Y rate + angle fig3.png")

# number of initial samples used to estimate constant gyro bias
bias_samples = 80  # ~2 s at ~40 Hz


def main():
    df = pd.read_csv(csv_path)

    # time vector from bag time, shifted to start at zero
    t = df["bag_t_sec"].to_numpy(float)

    # drop any repeated timestamps
    keep = np.diff(np.r_[t[:1], t]) > 0
    df = df.loc[keep].reset_index(drop=True)
    t = df["bag_t_sec"].to_numpy(float)
    t = t - t[0]

    for axis, col, out in [
        ("x", "gyro_x", out_x),
        ("y", "gyro_y", out_y),
    ]:
        rate = df[col].to_numpy(float)  # rad/s

        n0 = min(bias_samples, len(rate))
        bias = float(np.mean(rate[:n0]))
        rate_d = rate - bias

        # integrate to get angle (radians) with cumulative trapezoid
        dt = np.diff(t, prepend=t[0])
        angle = np.cumsum(0.5 * (rate_d + np.r_[rate_d[:1], rate_d[:-1]]) * dt)

        fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        axs[0].plot(t, rate, lw=1.2, label=f"gyro_{axis} (raw)")
        axs[0].plot(t, rate_d, lw=1.0, alpha=0.8, label=f"gyro_{axis} (bias removed)")
        axs[0].set_ylabel("rate (rad/s)")
        axs[0].set_title(f"gyro {axis}: rate and integrated angle")
        axs[0].grid(True)
        axs[0].legend(loc="upper right")

        axs[1].plot(t, angle, lw=1.5, label=f"angle_{axis} (integrated)")
        axs[1].set_xlabel("time (s)")
        axs[1].set_ylabel("angle (rad)")
        axs[1].grid(True)
        axs[1].legend(loc="upper left")

        fig.tight_layout()
        fig.savefig(out, dpi=300)
        plt.close(fig)

        print(f"saved {out}  (bias_{axis} = {bias:.5f} rad/s)")


if __name__ == "__main__":
    main()
