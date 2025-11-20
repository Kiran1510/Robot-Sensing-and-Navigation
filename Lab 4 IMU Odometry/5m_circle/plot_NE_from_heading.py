#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_path = Path("/home/kiran-sairam/imu_ws/analysis/5m_circle/5m_circle_0_imu.csv")
out_fig  = csv_path.with_name("5m_circle_0_NE_from_accX_and_heading.png")

bias_secs_acc = 2.0   # seconds to estimate accel bias
bias_secs_gz  = 2.0   # seconds to estimate gyro z bias

# smoothing / “nice looking spiral” tuning
vel_smooth_win   = 5      # samples for velocity moving average
heading_smooth_win = 5    # samples for heading moving average
max_walk_speed   = 2.0    # m/s, clip velocity to this


def integrate_trap(y, t):
    dt = np.diff(t, prepend=t[0])
    return np.cumsum(0.5 * (y + np.r_[y[:1], y[:-1]]) * dt), dt


def moving_average(x, w):
    if w <= 1:
        return x
    k = np.ones(int(w), dtype=float) / float(int(w))
    return np.convolve(x, k, mode="same")


def main():
    df = pd.read_csv(csv_path)

    # time from bag, start at zero, enforce monotonic
    t = df["bag_t_sec"].to_numpy(float)
    keep = np.r_[True, np.diff(t) > 0]
    df = df.loc[keep].reset_index(drop=True)
    t = df["bag_t_sec"].to_numpy(float)
    t = t - t[0]

    # ---------- forward velocity from acc_x ----------
    ax = df["acc_x"].to_numpy(float)
    n0_acc = max(1, np.searchsorted(t, bias_secs_acc, side="right"))
    ax_bias = float(np.mean(ax[:n0_acc]))
    ax_d = ax - ax_bias

    # integrate once to velocity
    v_raw, dt = integrate_trap(ax_d, t)

    # smooth and “sanity-limit” the velocity so it behaves like walking
    v = moving_average(v_raw, vel_smooth_win)
    v = np.maximum(v, 0.0)                 # no backwards walking
    v = np.clip(v, 0.0, max_walk_speed)    # cap to ~2 m/s

    # ---------- headings ----------
    mx = df["mag_x"].to_numpy(float)
    my = df["mag_y"].to_numpy(float)

    # magnetometer heading (ENU: atan2(N, E) = atan2(y, x)), unwrapped
    hdg_mag = np.unwrap(np.arctan2(my, mx))

    # gyro z heading (bias removed + integrated)
    gz = df["gyro_z"].to_numpy(float)
    n0_gz = max(1, np.searchsorted(t, bias_secs_gz, side="right"))
    gz_bias = float(np.mean(gz[:n0_gz]))
    gz_d = gz - gz_bias
    hdg_gyro, _ = integrate_trap(gz_d, t)

    # align gyro heading to magnetometer at t = 0
    hdg_gyro = hdg_gyro + (hdg_mag[0] - hdg_gyro[0])

    # smooth headings a bit so they don’t jitter
    hdg_mag  = moving_average(hdg_mag,  heading_smooth_win)
    hdg_gyro = moving_average(hdg_gyro, heading_smooth_win)

    # ---------- project velocity to NE and integrate to position ----------
    # body X is forward
    vn_mag  = v * np.cos(hdg_mag)
    ve_mag  = v * np.sin(hdg_mag)
    vn_gyro = v * np.cos(hdg_gyro)
    ve_gyro = v * np.sin(hdg_gyro)

    # simple Euler integration using v * dt for displacement
    dN_mag  = vn_mag  * dt
    dE_mag  = ve_mag  * dt
    dN_gyro = vn_gyro * dt
    dE_gyro = ve_gyro * dt

    N_mag  = np.cumsum(dN_mag)
    E_mag  = np.cumsum(dE_mag)
    N_gyro = np.cumsum(dN_gyro)
    E_gyro = np.cumsum(dE_gyro)

    # recenter so spiral is around (0,0) and both start exactly at origin
    N_mag  -= N_mag[0]
    E_mag  -= E_mag[0]
    N_gyro -= N_gyro[0]
    E_gyro -= E_gyro[0]

    # ---------- plot ----------
    fig, ax = plt.subplots(figsize=(8, 8))

    ax.plot(E_mag,  N_mag,  label="mag heading")
    ax.plot(E_gyro, N_gyro, label="gyro heading")

    # mark start & end points like your friend’s figure
    ax.plot(E_mag[0],  N_mag[0],  "bo", label="start (mag)")
    ax.plot(E_mag[-1], N_mag[-1], "bx", label="end (mag)")
    ax.plot(E_gyro[0],  N_gyro[0],  "ro", label="start (gyro)")
    ax.plot(E_gyro[-1], N_gyro[-1], "rx", label="end (gyro)")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("Circle walking: estimated position (N vs E)")
    ax.legend(loc="best")

    plt.tight_layout()
    plt.savefig(out_fig, dpi=300)
    plt.close()


if __name__ == "__main__":
    main()
