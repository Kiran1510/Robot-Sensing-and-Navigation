#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from mcap_ros2.reader import read_ros2_messages
from gps_driver.msg import Customrtk  #making msg discoverable

#config
OPEN_FILE = Path("/home/kiran-sairam/gnss/data/open_data_rtk/open_data_rtk_0.mcap")
OCCL_FILE = Path("/home/kiran-sairam/gnss/data/occluded_data_rtk/occluded_data_rtk_0.mcap")
TOPIC     = "/gps"
OUT_PNG   = Path("/home/kiran-sairam/gnss/analysis/stationary_rtk_position_histograms.png")

BINS = 40  #no of histogram elements

def utm_arrays(mcap_path: Path, topic: str):
    #read utm easting and northing arrays from mcap
    e, n = [], []
    with open(mcap_path, "rb") as f:
        for m in read_ros2_messages(f, topics=[topic]):
            msg = m.ros_msg
            e.append(float(msg.utm_easting))
            n.append(float(msg.utm_northing))
    return np.asarray(e), np.asarray(n)

def centroid_distances(e, n):
    #find distances to centroid in 2D in easting, northing
    ce, cn = np.mean(e), np.mean(n)
    return np.hypot(e - ce, n - cn)

#load data
e_open, n_open = utm_arrays(OPEN_FILE, TOPIC)
e_occl, n_occl = utm_arrays(OCCL_FILE, TOPIC)

#distances to centroid
d_open = centroid_distances(e_open, n_open)
d_occl = centroid_distances(e_occl, n_occl)

#some data
rms_open = float(np.sqrt(np.mean(d_open**2))) if d_open.size else 0.0
rms_occl = float(np.sqrt(np.mean(d_occl**2))) if d_occl.size else 0.0
mean_open = float(np.mean(d_open)) if d_open.size else 0.0
mean_occl = float(np.mean(d_occl)) if d_occl.size else 0.0

print(f"Open:  count={d_open.size}, mean={mean_open:.3f} m, RMS={rms_open:.3f} m")
print(f"Occl:  count={d_occl.size}, mean={mean_occl:.3f} m, RMS={rms_occl:.3f} m")

#plot
fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)

#taking 99th percentile
xmax = 0.0
if d_open.size: xmax = max(xmax, float(np.percentile(d_open, 99)))
if d_occl.size: xmax = max(xmax, float(np.percentile(d_occl, 99)))
xmax = max(xmax, 1e-6) * 1.05

#open histogram
axes[0].hist(d_open, bins=BINS, color="tab:blue", alpha=0.85)
axes[0].axvline(mean_open, color="tab:blue", linestyle="--", linewidth=1.2, label="mean")
axes[0].axvline(rms_open, color="tab:blue", linestyle=":", linewidth=1.2, label="rms")
axes[0].set_title("open sky, distance to centroid")
axes[0].set_xlabel("distance (m)")
axes[0].set_ylabel("count")
axes[0].set_xlim(0, xmax)
axes[0].grid(axis="y", alpha=0.3)
axes[0].text(0.98, 0.95,
             f"mean = {mean_open:.3f} m\nRMS = {rms_open:.3f} m",
             transform=axes[0].transAxes, ha="right", va="top",
             bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"))

#occluded histogram
axes[1].hist(d_occl, bins=BINS, color="tab:red", alpha=0.85)
axes[1].axvline(mean_occl, color="tab:red", linestyle="--", linewidth=1.2, label="mean")
axes[1].axvline(rms_occl, color="tab:red", linestyle=":", linewidth=1.2, label="rms")
axes[1].set_title("occluded sky, distance to centroid")
axes[1].set_xlabel("distance (m)")
axes[1].set_xlim(0, xmax)
axes[1].grid(axis="y", alpha=0.3)
axes[1].text(0.98, 0.95,
             f"mean = {mean_occl:.3f} m\nRMS = {rms_occl:.3f} m",
             transform=axes[1].transAxes, ha="right", va="top",
             bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"))

#legend and save
axes[0].legend(loc="upper right", fontsize=8)
axes[1].legend(loc="upper right", fontsize=8)
fig.tight_layout()
fig.savefig(OUT_PNG, dpi=250)
print(f"Saved: {OUT_PNG}")
