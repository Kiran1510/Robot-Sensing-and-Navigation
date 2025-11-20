#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# paths
csv_path = Path("/home/kiran-sairam/imu_ws/analysis/5m_circle/5m_circle_0_imu.csv")
out_png = csv_path.with_name("Magnetometer XY calibration fig1.png")

# tesla to milligauss
T_TO_MG = 1e7


def main():
    df = pd.read_csv(csv_path)

    # raw magnetometer data in milligauss
    mx = df["mag_x"].to_numpy(float) * T_TO_MG
    my = df["mag_y"].to_numpy(float) * T_TO_MG

    # hard-iron offsets
    bias_x = 0.5 * (mx.max() + mx.min())
    bias_y = 0.5 * (my.max() + my.min())
    mx_centered = mx - bias_x
    my_centered = my - bias_y

    # soft-iron scale (y only)
    max_x = np.max(np.abs(mx_centered))
    max_y = np.max(np.abs(my_centered))
    scale_y = max_x / max_y if max_y != 0 else 1.0

    mx_cal = mx_centered
    my_cal = my_centered * scale_y

    # print calibration parameters
    print("magnetometer xy calibration parameters")
    print(f"  bias_x (mG): {bias_x:.3f}")
    print(f"  bias_y (mG): {bias_y:.3f}")
    print(f"  scale_y:     {scale_y:.6f}")

    scale_matrix = np.array([[1.0, 0.0],
                             [0.0, scale_y]])
    print("soft-iron 2x2 scale matrix:")
    print(scale_matrix)

    # plot raw vs calibrated
    plt.figure(figsize=(7, 6))
    plt.scatter(mx, my, s=5, color="C0", label="raw")
    plt.scatter(mx_cal, my_cal, s=5, color="C3", label="calibrated")
    plt.scatter([bias_x], [bias_y], s=60, color="C0", edgecolor="k", label="raw center")
    plt.scatter([0], [0], s=60, color="C3", edgecolor="k", label="calibrated center")

    plt.gca().set_aspect("equal", adjustable="box")
    plt.title("magnetometer data before and after correction")
    plt.xlabel("magnetometer x (mG)")
    plt.ylabel("magnetometer y (mG)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_png, dpi=300)
    plt.close()

    print(f"saved calibration plot: {out_png}")


if __name__ == "__main__":
    main()
