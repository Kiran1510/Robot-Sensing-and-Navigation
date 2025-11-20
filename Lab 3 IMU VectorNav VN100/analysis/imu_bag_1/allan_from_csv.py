#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# -------- Allan deviation (simple, non-overlapping) --------
def allan_simple(series, dt, num_taus=200):
    s = np.asarray(series, float)
    N = len(s)
    max_tau = max(2, N // 2)
    taus_samp = np.unique(np.logspace(0, np.log10(max_tau), num_taus).astype(int))
    taus_out, adev_out = [], []

    for m in taus_samp:
        blocks = N // m
        if blocks < 2:
            continue
        avgs = s[:blocks*m].reshape(blocks, m).mean(axis=1)
        adev = np.sqrt(0.5 * np.mean(np.diff(avgs)**2))
        taus_out.append(m * dt)
        adev_out.append(adev)

    return np.array(taus_out), np.array(adev_out)

# -------- ARW, Bias, RRW --------
def N_at_1s(taus, adev):
    idx = np.argmin(np.abs(taus - 1.0))
    return float(adev[idx])

def estimate_K(taus, adev):
    k = max(5, int(0.2 * len(taus)))
    x = np.log(taus[-k:])
    y = np.log(adev[-k:])
    A = np.vstack([x, np.ones_like(x)]).T
    slope, intercept = np.linalg.lstsq(A, y, rcond=None)[0]
    K = np.sqrt(3) * np.exp(intercept)
    return float(slope), float(K)

def metrics(taus, adev):
    N = N_at_1s(taus, adev)
    B = float(np.min(adev))
    slope, K = estimate_K(taus, adev)
    return N, B, slope, K

# -------- Plot 3-in-1 Gyro Figure --------
def plot_gyro(taus_list, adev_list, outpng):
    titles = ["Gyro X", "Gyro Y", "Gyro Z"]
    fig, axes = plt.subplots(1, 3, figsize=(18,5))

    for i, ax in enumerate(axes):
        taus = taus_list[i]
        adev = adev_list[i]
        N, B, slope, K = metrics(taus, adev)

        ax.loglog(taus, adev, 'o-', markersize=3, linewidth=1.2, label="Allan dev")
        ax.axvline(1, color='g', linestyle='--', alpha=0.5)
        ax.axhline(B, color='r', linestyle='--', alpha=0.5)
        ax.set_title(titles[i])
        ax.set_xlabel("Tau (s)")
        ax.set_ylabel("Allan Deviation (rad/s)")
        ax.grid(True, which='both', alpha=0.3)

        ax.text(0.03, 0.05,
                f"N={N:.2e}\nB={B:.2e}\nK={K:.2e}",
                transform=ax.transAxes,
                fontsize=10,
                bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

    plt.tight_layout()
    outpng.parent.mkdir(exist_ok=True, parents=True)
    plt.savefig(outpng, dpi=200)
    plt.close()
    print(f"✅ Saved {outpng}")

# -------- Main --------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv", type=Path)
    args = p.parse_args()

    df = pd.read_csv(args.csv)

    # assume fixed 40 Hz sample rate
    dt = 1/40.0

    gyro_cols = ["gyro_x","gyro_y","gyro_z"]
    gyro_data = [df[c].to_numpy() for c in gyro_cols]

    gyro_taus, gyro_adev = [], []
    for arr in gyro_data:
        taus, adev = allan_simple(arr, dt, num_taus=200)
        gyro_taus.append(taus)
        gyro_adev.append(adev)

    outpng = args.csv.parent / "allan_gyro_3in1.png"
    plot_gyro(gyro_taus, gyro_adev, outpng)
    print("\n✅ Gyro Allan analysis complete\n")

if __name__ == "__main__":
    main()
