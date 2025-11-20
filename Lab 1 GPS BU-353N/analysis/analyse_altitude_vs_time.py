#!/usr/bin/env python3

import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages


def pick_mcap(path_str: str) -> Path:
    """Return a concrete .mcap path given a directory or file."""
    p = Path(path_str)
    if p.is_dir():
        files = sorted(p.glob("*.mcap"))
        if not files:
            raise FileNotFoundError(f"No .mcap in: {p}")
        return files[0]
    if p.is_file() and p.suffix == ".mcap":
        return p
    raise FileNotFoundError(f"Not a .mcap path: {p}")


def read_time_alt(mcap_path: Path, topic: str):
    """Return (t_minutes_relative, altitude_m) arrays from one bag."""
    ts, alts = [], []
    with open(mcap_path, "rb") as f:
        # topics=[topic] lets the reader filter for us
        for m in read_ros2_messages(f, topics=[topic]):
            msg = m.ros_msg
            t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            a = float(msg.altitude)
            ts.append(t)
            alts.append(a)

    if not ts:
        return np.array([]), np.array([])

    t = np.asarray(ts, dtype=float)
    a = np.asarray(alts, dtype=float)
    t_rel_min = (t - t.min()) / 60.0
    return t_rel_min, a


def main():
    ap = argparse.ArgumentParser(description="Altitude vs Time (open vs occluded)")
    ap.add_argument("--open", required=True, help="Open/clear-sky bag (.mcap or folder)")
    ap.add_argument("--occluded", required=True, help="Occluded bag (.mcap or folder)")
    ap.add_argument("--topic", default="/gps", help="Topic name (default: /gps)")
    ap.add_argument("--open-label", default="Open")
    ap.add_argument("--occluded-label", default="Occluded")
    ap.add_argument("--open-marker", default="o")
    ap.add_argument("--occluded-marker", default="x")
    args = ap.parse_args()

    open_mcap = pick_mcap(args.open)
    occ_mcap = pick_mcap(args.occluded)

    t_open, a_open = read_time_alt(open_mcap, args.topic)
    t_occ,  a_occ  = read_time_alt(occ_mcap, args.topic)

    if t_open.size == 0 and t_occ.size == 0:
        print("No messages found in either bag for that topic. Check paths/topic.")
        return

    plt.figure(figsize=(9, 5))
    if t_open.size:
        plt.scatter(t_open, a_open, s=14, marker=args.open_marker, label=args.open_label, alpha=0.8)
    else:
        print("[open] no data on that topic")

    if t_occ.size:
        plt.scatter(t_occ, a_occ, s=14, marker=args.occluded_marker, label=args.occluded_label, alpha=0.8)
    else:
        print("[occluded] no data on that topic")

    plt.xlabel("Time (minutes, relative)")
    plt.ylabel("Altitude (m)")
    plt.title("Altitude vs Time — Open vs Occluded")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

