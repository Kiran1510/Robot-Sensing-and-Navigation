#!/usr/bin/env python3

import argparse
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import utm
from mcap_ros2.reader import read_ros2_messages

def pick_mcap(path):
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "*.mcap")))
        if not files:
            raise FileNotFoundError(f"no .mcap file found in: {path}")
        return files[0]
    return path

def read_e_n(path, topic):
    e, n = [], []
    mcap = pick_mcap(path)
    with open(mcap, "rb") as f:
        for m in read_ros2_messages(f, topics=[topic]):
            msg = m.ros_msg
            e.append(float(msg.utm_easting))
            n.append(float(msg.utm_northing))
    return np.array(e, dtype=float), np.array(n, dtype=float)

def distances_to_centroid(e, n):
    if e.size == 0:
        return np.array([]), (np.nan, np.nan)
    ce = float(np.mean(e))
    cn = float(np.mean(n))
    d = np.sqrt((e - ce)**2 + (n - cn)**2)
    return d, (ce, cn)

def plot_hist(distances, title, bins):
    if distances.size == 0:
        print(f"[!] no data for: {title}")
        return
    mean = float(np.mean(distances))
    std  = float(np.std(distances))
    plt.figure(figsize=(8,5))
    plt.hist(distances, bins=bins, alpha=0.85)
    plt.title(title)
    plt.xlabel("Distance from centroid (m)")
    plt.ylabel("Count")
    plt.grid(True, axis="y", alpha=0.3)
    plt.legend(loc="upper left")
    plt.tight_layout()

def main():
    ap = argparse.ArgumentParser(description="stationary position-error histograms with reference coordinates")
    ap.add_argument("--open", required=True, help="open/clear-sky bag (.mcap or folder)")
    ap.add_argument("--occluded", required=True, help="occluded bag (.mcap or folder)")
    ap.add_argument("--topic", default="/gps", help="topic name (default: /gps)")
    ap.add_argument("--open-ref-lat", type=float, help="reference latitude for open area")
    ap.add_argument("--open-ref-lon", type=float, help="reference longitude for open area")
    ap.add_argument("--occluded-ref-lat", type=float, help="reference latitude for occluded area")
    ap.add_argument("--occluded-ref-lon", type=float, help="reference longitude for occluded area")
    ap.add_argument("--bins", type=int, default=40, help="histogram bins (default: 40)")
    args = ap.parse_args()

    # open sky
    e_open, n_open = read_e_n(args.open, args.topic)
    d_open, (ce_o, cn_o) = distances_to_centroid(e_open, n_open)
    print("--- OPEN SKY ---")
    print(f"centroid UTM: E={ce_o:.3f}, N={cn_o:.3f}")
    if args.open_ref_lat and args.open_ref_lon:
        open_ref_e, open_ref_n, _, _ = utm.from_latlon(args.open_ref_lat, args.open_ref_lon)
        print(f"reference UTM: E={open_ref_e:.3f}, N={open_ref_n:.3f}")
    if d_open.size:
        print(f"mean={np.mean(d_open):.2f} m, std={np.std(d_open):.2f} m")
    plot_hist(d_open, "Position Error — Open/Clear", args.bins)

    # occluded
    e_occ, n_occ = read_e_n(args.occluded, args.topic)
    d_occ, (ce_c, cn_c) = distances_to_centroid(e_occ, n_occ)
    print("\n--- OCCLUDED ---")
    print(f"centroid UTM: E={ce_c:.3f}, N={cn_c:.3f}")
    if args.occluded_ref_lat and args.occluded_ref_lon:
        occ_ref_e, occ_ref_n, _, _ = utm.from_latlon(args.occluded_ref_lat, args.occluded_ref_lon)
        print(f"reference UTM: E={occ_ref_e:.3f}, N={occ_ref_n:.3f}")
    if d_occ.size:
        print(f"mean={np.mean(d_occ):.2f} m, std={np.std(d_occ):.2f} m")
    plot_hist(d_occ, "Position Error — Occluded", args.bins)

    plt.show()

if __name__ == "__main__":
    main()

