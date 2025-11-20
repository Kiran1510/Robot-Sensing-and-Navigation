#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages
import utm

def read_utm_data(bag_path, topic):
    """Reads GPS data from the .mcap and converts lat/lon to UTM coordinates."""
    easting, northing = [], []
    with open(bag_path, "rb") as f:
        for m in read_ros2_messages(f):
            topic_name = getattr(m, "topic", None) or getattr(m.channel, "topic", None)
            if topic_name == topic:
                gps = m.ros_msg
                e, n, _, _ = utm.from_latlon(gps.latitude, gps.longitude)
                easting.append(e)
                northing.append(n)
    return np.array(easting), np.array(northing)

def fit_line(e, n):
    """Fits a best-fit line (northing vs easting) and computes perpendicular distances."""
    A = np.vstack([e, np.ones_like(e)]).T
    slope, intercept = np.linalg.lstsq(A, n, rcond=None)[0]
    n_pred = slope * e + intercept
    dist = np.abs(slope * e - n + intercept) / np.sqrt(slope**2 + 1)
    return slope, intercept, n_pred, dist

def main():
    parser = argparse.ArgumentParser(description="Moving path with best-fit line")
    parser.add_argument("--bag", required=True, help="Path to .mcap file")
    parser.add_argument("--topic", default="/gps", help="Topic name (default: /gps)")
    parser.add_argument("--label", default="Walking Path", help="Label for legend")
    parser.add_argument("--marker", default="o", help="Marker style (e.g. o, x, .)")
    args = parser.parse_args()

    e, n = read_utm_data(args.bag, args.topic)
    if len(e) < 3:
        print("Not enough GPS points to analyze.")
        return

    slope, intercept, n_pred, dist = fit_line(e, n)

    # simple terminal stats
    print(f"Total samples: {len(e)}")
    print(f"Line equation: n = {slope:.6f} * e + {intercept:.2f}")
    print(f"Average deviation: {np.mean(dist):.2f} m")
    print(f"Max deviation: {np.max(dist):.2f} m")

    # plot points + best-fit line
    plt.figure(figsize=(8,6))
    plt.scatter(e, n, s=15, marker=args.marker, alpha=0.8, label=args.label)
    plt.plot(e, n_pred, color="red", linewidth=1.3, label="Best-fit line")

    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("Walking Path with Best-Fit Line")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

