# Robot Sensing and Navigation

Some of the work I did for the **EECE5554 - Robot Sensing and Navigation** course at Northeastern University.

## Course Overview

This repository contains my approach to projects involving, among several other things, writing Python drivers in ROS2 for various sensors and navigation systems used in robotics, including GPS, RTK GPS, IMU sensors, and ROS2 implementations. Each lab includes comprehensive technical reports documenting methodologies, analysis, and results.

## Repository Structure

### Lab Assignments

- **Lab 0: ROS/Linux/Git Basics**
  - Introduction to ROS2, Linux environment, and version control with Git

- **Lab 1: GPS BU-353N**
  - GPS data collection and analysis using the BU-353N sensor
  - Position accuracy evaluation and error characterization
  - Statistical analysis of stationary and moving data
  - **[Lab Report Available](Lab%201%20GPS%20BU-353N/)** - Detailed quiz responses with error analysis and visualization

- **Lab 2: RTK GPS**
  - Real-Time Kinematic (RTK) GPS positioning
  - High-precision positioning with centimeter-level accuracy
  - Analysis of stationary and moving RTK data
  - Comparison between standard GPS and RTK performance
  - **[Lab Report Available](Lab%202%20RTK%20GPS/)** - Comprehensive report on RTK vs. standalone GPS performance

- **Lab 3: IMU VectorNav VN100**
  - Inertial Measurement Unit (IMU) sensor integration
  - Orientation and angular velocity measurements
  - Sensor calibration and error analysis
  - Allan Deviation analysis for sensor characterization
  - **[Lab Report Available](Lab%203%20IMU%20VectorNav%20VN100/)** - Quiz documentation with IMU data analysis and video synchronization

- **Lab 4: IMU Odometry**
  - Dead reckoning using IMU data
  - Trajectory reconstruction from inertial measurements
  - Magnetometer calibration (hard-iron and soft-iron correction)
  - Gyroscope and accelerometer integration
  - **[Lab Report Available](Lab%204%20IMU%20Odometry/)** - Complete analysis of circle and square walking patterns

- **Lab 5: Automotive Dead Reckoning**
  - Integration of VectorNav VN-100 IMU and BU-353N GPS for vehicle navigation
  - Multi-sensor fusion for improved positioning accuracy
  - Dead reckoning implementation for automotive applications
  - Complementary filtering and sensor fusion techniques
  - **[Lab Report Available](Lab%205%20Automotive%20Dead%20Reckoning/)** - Automotive sensor fusion analysis

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
- Statistical analysis and error characterization
- Dead reckoning and odometry

## Documentation

Each lab folder contains:
- **Technical Reports (PDF)** - Comprehensive reports with analysis and results
- **Source Code** - Python scripts for data collection and analysis
- **Data Visualization** - Plots and figures illustrating sensor performance
- **Analysis Scripts** - Tools for processing ROS2 bag files and sensor data

## Getting Started

### Prerequisites

- ROS2 (Jazzy)
- Python 3.x
- Required Python packages: numpy, matplotlib, pandas

### Running the Code

Each lab folder contains its own scripts and launch files. Navigate to the directory, download, open each analysis script and change the export location to wherever you wish to save the plots. (By default, it is set to my computer's directory)

## Analysis & Results

Each lab includes:
- Comprehensive technical reports in PDF format
- Data collected and analyzed from actual sensor experiments
- Analysis scripts for processing ROS2 bag files
- Plots and visualizations of sensor performance
- Performance metrics and error analysis
- Detailed observations and conclusions
- Comparisons with theoretical expectations

## Highlights

- **GPS Analysis**: Statistical error characterization with RMS calculations and HDOP analysis
- **RTK GPS**: Demonstrated centimeter-level accuracy in open environments
- **IMU Calibration**: Hard-iron and soft-iron magnetometer correction
- **Dead Reckoning**: Integration-based position estimation with drift analysis
- **Sensor Fusion**: Multi-sensor integration for improved navigation accuracy

## Author

**Kiran Sairam Bethi Balagangadaran**  
MS Robotics, Northeastern University

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Northeastern University EECE5554 Course Staff
- ROS2 Community
- Open-source sensor driver contributors
