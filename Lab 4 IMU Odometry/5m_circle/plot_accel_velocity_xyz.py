#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# paths
csv_path = Path("/home/kiran-sairam/imu_ws/analysis/5m_circle/5m_circle_0_imu.csv")

out_x = csv_path.with_name("Accel X + velocity X fig5.png")
out_y = csv_path.with_name("Accel Y + velocity Y fig6.png")
out_z = csv_path.with_name("Accel Z + velocity Z fig7.png")

# seconds used to estimate constant accel bias
bias_seconds = 2.0


def load_time(df: pd.DataFrame) -> np.ndarray:
    # use bag time and shift so it starts at zero
    t = df["bag_t_sec"].to_numpy(float)
    t = t - t[0]
    return t


def estimate_bias(a: np.ndarray, t: np.ndarray, window: float) -> float:
    if window <= 0:
        return 0.0
    n0 = np.searchsorted(t, window, side="right")
    n0 = max(1, n0)
    return float(np.mean(a[:n0]))


def integrate_trapz(a: np.ndarray, t: np.ndarray) -> np.ndarray:
    dt = np.diff(t, prepend=t[0])
    return np.cumsum(0.5 * (a + np.r_[a[:1], a[:-1]]) * dt)


def plot_axis(t: np.ndarray, acc_raw: np.ndarray, axis: str, out_path: Path):
    bias = estimate_bias(acc_raw, t, bias_seconds)
    acc = acc_raw - bias
    vel = integrate_trapz(acc, t)

    fig, axs = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    axs[0].plot(t, acc_raw, lw=1.0, label=f"acc_{axis} (raw)")
    axs[0].plot(t, acc, lw=1.2, label=f"acc_{axis} (bias removed)")
    axs[0].set_ylabel("acceleration (m/s²)")
    axs[0].set_title(f"acceleration and velocity – axis {axis}")
    axs[0].grid(True)
    axs[0].legend(loc="upper right")

    axs[1].plot(t, vel, lw=1.6, label=f"vel_{axis} (integrated)")
    axs[1].set_xlabel("time (s)")
    axs[1].set_ylabel("velocity (m/s)")
    axs[1].grid(True)
    axs[1].legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)

    print(f"saved {out_path}  (bias_{axis} = {bias:.5f} m/s²)")


def main():
    df = pd.read_csv(csv_path)
    t = load_time(df)

    plot_axis(t, df["acc_x"].to_numpy(float), "x", out_x)
    plot_axis(t, df["acc_y"].to_numpy(float), "y", out_y)
    plot_axis(t, df["acc_z"].to_numpy(float), "z", out_z)


if __name__ == "__main__":
    main()
