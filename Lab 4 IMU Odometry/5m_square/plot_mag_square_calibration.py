#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = Path("/home/kiran-sairam/imu_ws/analysis/5m_square/5m_square_0_imu.csv")
out_png = csv_path.with_name("Mag XY, raw vs calibrated fig9.png")
out_txt = csv_path.with_name("Mag XY, raw vs calibrated.txt")

to_mg = 1e7  # tesla -> milligauss


def load_xy(path):
    df = pd.read_csv(path)
    x = df["mag_x"].to_numpy(float)
    y = df["mag_y"].to_numpy(float)
    return x, y


def hardiron_center(x, y):
    cx = 0.5 * (np.nanmax(x) + np.nanmin(x))
    cy = 0.5 * (np.nanmax(y) + np.nanmin(y))
    return np.array([cx, cy])


def softiron_matrix(centered_xy):
    # centered_xy: 2xN
    s = (centered_xy @ centered_xy.T) / centered_xy.shape[1]
    vals, vecs = np.linalg.eigh(s)
    inv_sqrt = vecs @ np.diag(1.0 / np.sqrt(vals)) @ vecs.T
    yw = inv_sqrt @ centered_xy
    r_raw = np.mean(np.linalg.norm(centered_xy, axis=0))
    r_w = np.mean(np.linalg.norm(yw, axis=0))
    k = r_raw / r_w if r_w > 0 else 1.0
    m = k * inv_sqrt
    return m


def main():
    x, y = load_xy(csv_path)
    xy = np.vstack([x, y])  # 2xN

    bias = hardiron_center(x, y)
    xy_centered = xy - bias.reshape(2, 1)

    m = softiron_matrix(xy_centered)
    xy_cal = m @ xy_centered

    with open(out_txt, "w") as f:
        f.write("# magnetometer xy calibration (hard-iron + soft-iron)\n")
        f.write(f"bias_bx_by = [{bias[0]:.8e}, {bias[1]:.8e}]  # tesla\n")
        f.write("softiron_M =\n")
        for row in m:
            f.write(f"  [{row[0]:.8e}, {row[1]:.8e}]\n")

    plt.figure(figsize=(8.5, 8.0))
    plt.scatter(xy[0] * to_mg, xy[1] * to_mg, s=6, alpha=0.45, label="raw")
    plt.scatter(xy_cal[0] * to_mg, xy_cal[1] * to_mg, s=6, alpha=0.75, label="calibrated")
    plt.scatter([bias[0] * to_mg], [bias[1] * to_mg], s=60, c="k", marker="x", label="raw center")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.grid(True, alpha=0.3)
    plt.xlabel("mag x (mG)")
    plt.ylabel("mag y (mG)")
    plt.title("magnetometer xy: raw vs calibrated")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(out_png, dpi=300)
    plt.close()


if __name__ == "__main__":
    main()
