#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from mcap_ros2.reader import read_ros2_messages
from gps_driver.msg import Customrtk  #making msg discoverable

#config
WALK_FILE = Path("/home/kiran-sairam/gnss/data/walking_data_rtk/walking_data_rtk_0.mcap")
TOPIC     = "/gps"
OUT_PNG   = Path("/home/kiran-sairam/gnss/analysis/moving_rtk_altitude_vs_time.png")

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
t_w, a_w = time_alt(WALK_FILE, TOPIC)
if t_w.size == 0:
    raise SystemExit("No GPS altitude data found in walking_data_rtk.")

#plot
plt.figure(figsize=(12, 6))
plt.scatter(t_w, a_w, s=40, alpha=0.9, color="tab:blue", marker="o", label="walking (rtk)")

plt.grid(True, linestyle="--", alpha=0.35)
plt.xlabel("time (minutes, relative)")
plt.ylabel("altitude (m)")
plt.title("moving rtk: altitude vs time")
plt.legend(loc="best")

#define plot limits
pad_t = 0.02 * (t_w.max() - t_w.min() + 1e-6)
pad_a = 0.05 * (a_w.max() - a_w.min() + 1e-6)
plt.xlim(t_w.min() - pad_t, t_w.max() + pad_t)
plt.ylim(a_w.min() - pad_a, a_w.max() + pad_a)

plt.tight_layout()
plt.savefig(OUT_PNG, dpi=250)
print(f"Saved: {OUT_PNG}")
