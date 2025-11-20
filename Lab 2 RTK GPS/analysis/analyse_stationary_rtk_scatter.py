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
OUT_PNG   = Path("/home/kiran-sairam/gnss/analysis/stationary_rtk_northing_vs_easting.png")

def utm_arrays(mcap_path: Path, topic: str):
    e, n = [], []
    with open(mcap_path, "rb") as f:
        for m in read_ros2_messages(f, topics=[topic]):
            msg = m.ros_msg
            e.append(float(msg.utm_easting))
            n.append(float(msg.utm_northing))
    return np.asarray(e), np.asarray(n)

def metrics(e_rel, n_rel):
    r = np.hypot(e_rel, n_rel)
    return float(np.sqrt(np.mean(r*r))), float(np.mean(r))  # rms, mean

#load and center
e_open, n_open = utm_arrays(OPEN_FILE, TOPIC)
e_occl, n_occl = utm_arrays(OCCL_FILE, TOPIC)
ce_open, cn_open = np.mean(e_open), np.mean(n_open)
ce_occl, cn_occl = np.mean(e_occl), np.mean(n_occl)
e_open_rel, n_open_rel = e_open - ce_open, n_open - cn_open
e_occl_rel, n_occl_rel = e_occl - ce_occl, n_occl - cn_occl

#offsets and stats
delta_e, delta_n = ce_occl - ce_open, cn_occl - cn_open
rms_open, mean_open = metrics(e_open_rel, n_open_rel)
rms_occl, mean_occl = metrics(e_occl_rel, n_occl_rel)

print(f"Open:  RMS={rms_open:.3f} m, Mean={mean_open:.3f} m")
print(f"Occl:  RMS={rms_occl:.3f} m, Mean={mean_occl:.3f} m")
print(f"Centroid ΔE={delta_e:.3f} m, ΔN={delta_n:.3f} m")

#plot
plt.figure(figsize=(8, 8))
plt.scatter(e_open_rel, n_open_rel, s=40, alpha=0.7, color="tab:blue", marker="o", label="open sky")
plt.scatter(e_occl_rel, n_occl_rel, s=40, alpha=0.7, color="tab:red",  marker="x", label="occluded sky")
plt.axhline(0, color='gray', ls='--', lw=0.8)
plt.axvline(0, color='gray', ls='--', lw=0.8)
plt.xlabel("easting offset (m)"); plt.ylabel("northing offset (m)")
plt.title("stationary rtk: northing vs easting (centroid centered)")

#zoom into plot
margin = 0.2
x_min = min(e_open_rel.min(), e_occl_rel.min()) - margin
x_max = max(e_open_rel.max(), e_occl_rel.max()) + margin
y_min = min(n_open_rel.min(), n_occl_rel.min()) - margin
y_max = max(n_open_rel.max(), n_occl_rel.max()) + margin
plt.xlim(x_min, x_max)
plt.ylim(y_min, y_max)
# -------------------------------------------------------------------

#centroid markers placed near origin
plt.scatter(0, 0, s=120, color="tab:blue", edgecolor='black', marker='*', label="open centroid", zorder=5)
plt.scatter(0.1, -0.1, s=120, color="tab:red", edgecolor='black', marker='P', label="occluded centroid", zorder=5)

plt.legend(loc="upper left")

#annotation box
txt = f"ΔE = {delta_e:.3f} m\nΔN = {delta_n:.3f} m\nRMS(Open) = {rms_open:.3f} m\nRMS(Occl) = {rms_occl:.3f} m"
plt.text(0.98, 0.98, txt, transform=plt.gca().transAxes, ha='right', va='top',
         fontsize=10, bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

plt.tight_layout()
plt.savefig(OUT_PNG, dpi=250)
print(f"Saved: {OUT_PNG}")
