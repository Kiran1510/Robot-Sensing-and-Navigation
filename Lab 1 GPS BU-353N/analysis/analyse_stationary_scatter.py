#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages

def read_e_n_from_bag(mcap_path, topic_name):
    easts = []
    norths = []
    with open(mcap_path, "rb") as f:
        for m in read_ros2_messages(f):
            if getattr(m.channel, "topic", "") == topic_name:
                msg = m.ros_msg
                easts.append(float(msg.utm_easting))
                norths.append(float(msg.utm_northing))
    return np.array(easts), np.array(norths)

def main():
    parser = argparse.ArgumentParser(description="compare open vs occluded GPS scatter")
    parser.add_argument("--open", required=True, help="path to open-sky .mcap file")
    parser.add_argument("--occluded", required=True, help="path to occluded .mcap file")
    parser.add_argument("--topic", default="/gps", help="ROS2 topic name (default: /gps)")
    parser.add_argument("--open-label", default="Open Sky", help="label for open sky data")
    parser.add_argument("--occluded-label", default="Occluded", help="label for occluded data")
    parser.add_argument("--open-marker", default="o", help="marker for open data")
    parser.add_argument("--occluded-marker", default="x", help="marker for occluded data")
    args = parser.parse_args()

    # read both files
    e_open, n_open = read_e_n_from_bag(args.open, args.topic)
    e_occ, n_occ = read_e_n_from_bag(args.occluded, args.topic)

    if len(e_open) == 0 or len(e_occ) == 0:
        print("Error: one of the files had no GPS data")
        return

    # find centroids
    e_open_c, n_open_c = np.mean(e_open), np.mean(n_open)
    e_occ_c, n_occ_c = np.mean(e_occ), np.mean(n_occ)

    # center around (0,0)
    e_open_rel = e_open - e_open_c
    n_open_rel = n_open - n_open_c
    e_occ_rel = e_occ - e_occ_c
    n_occ_rel = n_occ - n_occ_c

    # plot
    plt.figure(figsize=(8,6))
    plt.scatter(e_open_rel, n_open_rel, s=15, marker=args.open_marker, alpha=0.7, label=args.open_label)
    plt.scatter(e_occ_rel, n_occ_rel, s=15, marker=args.occluded_marker, alpha=0.7, label=args.occluded_label)

    plt.axhline(0, color="gray", lw=0.8)
    plt.axvline(0, color="gray", lw=0.8)
    plt.gca().set_aspect("equal", adjustable="box")

    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("Stationary GPS Scatter (Centered by Centroid)")
    plt.legend()
    plt.grid(True, ls="--", alpha=0.4)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

