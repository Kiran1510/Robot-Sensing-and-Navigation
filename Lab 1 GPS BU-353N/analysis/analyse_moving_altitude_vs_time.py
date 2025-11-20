#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from mcap_ros2.reader import read_ros2_messages

def pick_mcap(path_str):
    p = Path(path_str)
    if p.is_dir():
        files = sorted(p.glob("*.mcap"))
        if not files:
            raise FileNotFoundError(f"no .mcap file in {p}")
        return files[0]
    elif p.is_file() and p.suffix == ".mcap":
        return p
    else:
        raise FileNotFoundError(f"invalid path: {p}")

def read_alt_time(mcap_path, topic):
    times = []
    alts = []
    with open(mcap_path, "rb") as f:
        for m in read_ros2_messages(f, topics=[topic]):
            msg = m.ros_msg
            t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            times.append(t)
            alts.append(float(msg.altitude))
    if not times:
        raise RuntimeError(f"no data on {topic}")
    t = np.array(times)
    a = np.array(alts)
    t_rel = (t - t.min()) / 60.0  # convert to minutes
    return t_rel, a

def main():
    ap = argparse.ArgumentParser(description="Altitude vs Time (one bag)")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/gps")
    ap.add_argument("--label", default="Walking Path")
    ap.add_argument("--marker", default="o")
    ap.add_argument("--save", default=None)
    args = ap.parse_args()

    mcap = pick_mcap(args.bag)
    t, alt = read_alt_time(mcap, args.topic)

    plt.figure(figsize=(8,5))
    plt.scatter(t, alt, s=15, marker=args.marker, alpha=0.8, label=args.label)
    plt.xlabel("Time (min)")
    plt.ylabel("Altitude (m)")
    plt.title("Altitude vs Time")
    plt.grid(True, ls="--", alpha=0.4)
    plt.legend()
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches="tight")
    plt.show()

if __name__ == "__main__":
    main()

