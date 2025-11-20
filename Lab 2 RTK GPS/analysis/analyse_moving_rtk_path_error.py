#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from mcap_ros2.reader import read_ros2_messages
from gps_driver.msg import Customrtk  #making msg discoverable

#config
WALK_FILE = Path("/home/kiran-sairam/gnss/data/walking_data_rtk/walking_data_rtk_0.mcap")
TOPIC     = "/gps"
OUT_PNG   = Path("/home/kiran-sairam/gnss/analysis/moving_rtk_path_with_bestfit.png")

def utm_arrays(mcap_path: Path, topic: str):
    #read utm easting and northing arrays from mcap
    e, n = [], []
    with open(mcap_path, "rb") as f:
        for m in read_ros2_messages(f, topics=[topic]):
            msg = m.ros_msg
            e.append(float(msg.utm_easting))
            n.append(float(msg.utm_northing))
    return np.asarray(e), np.asarray(n)

def fit_line(e, n):
    #fit n = s*e + b using least squares
    s, b = np.polyfit(e, n, 1)
    return float(s), float(b)

def orthogonal_errors(e, n, s, b):
    #perpendicular distance from each (e,n) to line n = s*e + b
    denom = np.sqrt(s*s + 1.0)
    return np.abs(s*e - n + b) / denom

#load data
e, n = utm_arrays(WALK_FILE, TOPIC)
if e.size < 3:
    raise SystemExit("not enough points in walking_data_rtk to fit line.")

#fit and errors
s, b = fit_line(e, n)
d = orthogonal_errors(e, n, s, b)

#basic data on error
mean_err = float(np.mean(d))
rms_err  = float(np.sqrt(np.mean(d*d)))
max_err  = float(np.max(d))
print(f"Best-fit line: n = {s:.6f} * e + {b:.3f}")
print(f"Orthogonal error to line -> mean={mean_err:.3f} m, RMS={rms_err:.3f} m, max={max_err:.3f} m")

#plot
plt.figure(figsize=(10, 8))

#scatter path
plt.scatter(e, n, s=18, alpha=0.9, color="tab:blue", marker="o", label="walking path")

#best-fit line
order = np.argsort(e)
e_fit = e[order]
n_fit = s*e_fit + b
plt.plot(e_fit, n_fit, color="tab:red", linewidth=2.0, label="best fit line")

#labels and aesthetics
plt.xlabel("easting (m)")
plt.ylabel("northing (m)")
plt.title("walking rtk: northing vs easting with best fit line")
plt.grid(True, linestyle="--", alpha=0.35)
plt.legend(loc="best")

#annotation box
txt = f"Mean dev. = {mean_err:.3f} m\nRMS dev. = {rms_err:.3f} m"
plt.text(0.98, 0.02, txt, transform=plt.gca().transAxes,
         ha='right', va='bottom', fontsize=10,
         bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

#set plot limits
pad_x = 0.02 * (e.max() - e.min() + 1e-6)
pad_y = 0.02 * (n.max() - n.min() + 1e-6)
plt.xlim(e.min() - pad_x, e.max() + pad_x)
plt.ylim(n.min() - pad_y, n.max() + pad_y)

plt.tight_layout()
plt.savefig(OUT_PNG, dpi=250)
print(f"Saved: {OUT_PNG}")
