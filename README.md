# EECE5554
A private repository for the Robotics Sensing and Navigation lab.
# IMU (EECE5554)
This repo contains:
- `IMU_analysis/` – scripts & plots
- `IMU_ws`/`imu_ws` – ROS 2 workspace snippets
- Large raw data (`*.mcap`, raw `*.csv`) could initially **not** be committed because they're larger than 100MB, but I manually uploaded them later

## Re-generate CSV from MCAP
python3 convert_mcap_to_csv.py

## Allan variance (gyro)
cd IMU_analysis/imu_bag_1
python3 allan_from_csv.py imu_bag_1_0_imu.csv
