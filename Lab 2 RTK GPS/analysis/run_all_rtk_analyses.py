#!/usr/bin/env python3
import subprocess
from pathlib import Path

ANALYSIS_DIR = Path("/home/kiran-sairam/gnss/analysis")
scripts = [
    "analyse_stationary_rtk_scatter.py",
    "analyse_stationary_rtk_altitude.py",
    "analyse_stationary_rtk_position_hist.py",
    "analyse_moving_rtk_path_error.py",
    "analyse_moving_rtk_altitude.py"
]

print("\n========== running all rtk analyses ==========\n")
for s in scripts:
    p = ANALYSIS_DIR / s
    if p.exists():
        print(f"{s}")
        subprocess.run(["python3", str(p)], check=False)
    else:
        print(f"missing: {s}")
print("\n all analyses complete")
