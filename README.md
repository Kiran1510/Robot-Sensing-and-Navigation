# Robot Sensing and Navigation

Course materials and lab assignments for **EECE5554 - Robot Sensing and Navigation** at Northeastern University.

## Course Overview

This repository contains my approach at projects exploring various sensors and navigation systems used in robotics, including GPS, RTK GPS, IMU sensors, and ROS2 implementations.

## Repository Structure

### Lab Assignments

- **Lab 0: ROS/Linux/Git Basics**
  - Introduction to ROS2, Linux environment, and version control with Git

- **Lab 1: GPS BU-353N**
  - GPS data collection and analysis using the BU-353N sensor
  - Position accuracy evaluation and error characterization using python scripts for plotting

- **Lab 2: RTK GPS**
  - Real-Time Kinematic (RTK) GPS positioning
  - High-precision positioning with centimeter-level accuracy
  - Analysis of stationary and moving RTK data
  - Comparison between standard GPS and RTK performance

- **Lab 3: IMU VectorNav VN100**
  - Inertial Measurement Unit (IMU) sensor integration
  - Orientation and angular velocity measurements
  - Sensor calibration and error analysis

- **Lab 4: IMU Odometry**
  - Dead reckoning using IMU data
  - Trajectory reconstruction from inertial measurements
  - Allan Deviation analysis for gyroscope characterization

## Technologies Used

- **ROS2** - Robot Operating System for sensor data processing
- **Python** - Data analysis and visualization
- **GNSS/GPS** - Global positioning systems
- **IMU Sensors** - Inertial measurement units for orientation tracking

## Key Concepts

- Sensor fusion and integration
- Position and orientation estimation
- Error analysis and calibration
- Real-time data processing
- ROS2 driver development

## Getting Started

### Prerequisites

- ROS2 (Jazzy)
- Python 3.x
- Required Python packages: numpy, matplotlib, pandas

### Running the Code

Each lab folder contains its own scripts and launch files. Navigate to the directory, download, open each analysis script and change the export location to wherever you wish to save the plots. (By default, it is set to my computer's directory)

## Analysis & Results

Each lab includes:
- Data collected/analysed
- Analysis scripts
- Plots 
- Performance metrics and errors
- Observations (where applicable)


## Author

**Kiran Sairam Bethi Balagangadaran**  
MS Robotics, Northeastern University

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Northeastern University EECE5554 Course Staff
- ROS2 Community
- Open-source sensor driver contributors
