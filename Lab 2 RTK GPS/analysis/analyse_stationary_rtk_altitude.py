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
OUT_PNG   = Path("/home/kiran-sairam/gnss/analysis/stationary_rtk_altitude_vs_time.png")

def time_alt(mcap_path: Path, topic: str):
    #read header time and altitude from mcap file
    t, a = [], []
    with open(mcap_path, "rb") as f:
        for m in read_ros2_messages(f, topics=[topic]):
            msg = m.ros_msg
            ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            t.append(ts)
            a.append(float(msg.altitude))
    if not t:
        return np.array([]), np.array([])
    t = np.asarray(t); a = np.asarray(a)
    order = np.argsort(t)
    t = t[order]; a = a[order]
    return (t - t[0]) / 60.0, a  #relative minutes, altitude meters

#load data
t_open, a_open = time_alt(OPEN_FILE, TOPIC)
t_occl, a_occl = time_alt(OCCL_FILE, TOPIC)

#plot
plt.figure(figsize=(12, 6))
plt.scatter(t_open, a_open, s=40, alpha=0.9, color="tab:blue", marker="o", label="open sky")
plt.scatter(t_occl, a_occl, s=40, alpha=0.9, color="tab:red",  marker="x", label="occluded sky")

plt.grid(True, linestyle="--", alpha=0.35)
plt.xlabel("time (minutes, relative)")
plt.ylabel("altitude (m)")
plt.title("stationary rtk: altitude vs time (open vs occluded)")
plt.legend(loc="best")

#make x/y limits around data
if t_open.size or t_occl.size:
    all_t = np.concatenate([t_open, t_occl]) if t_open.size and t_occl.size else (t_open if t_open.size else t_occl)
    all_a = np.concatenate([a_open, a_occl]) if a_open.size and a_occl.size else (a_open if a_open.size else a_occl)
    pad_t = 0.02 * (all_t.max() - all_t.min() + 1e-6)
    pad_a = 0.05 * (all_a.max() - all_a.min() + 1e-6)
    plt.xlim(all_t.min() - pad_t, all_t.max() + pad_t)
    plt.ylim(all_a.min() - pad_a, all_a.max() + pad_a)

plt.tight_layout()
plt.savefig(OUT_PNG, dpi=250)
print(f"Saved: {OUT_PNG}")
