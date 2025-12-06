# YB Car Robot - ROS2 Sensor Fusion Workspace

## Project Overview
This workspace provides sensor fusion and pose estimation for the YB_Car robot using ROS2 and robot_localization package.

## Robot Configuration
- **Robot Name**: YB_Car
- **Main Node**: /YB_Car_Node
- **Sensors**: IMU, Odometry, LaserScan, Battery
- **Purpose**: Sensor fusion for SLAM-ready pose estimation

## Key Components
- `yb_car_localization`: Main package for sensor fusion using EKF
- URDF robot description (to be provided by user)
- Launch files for sensor fusion and visualization

## Development Notes
- Uses robot_localization EKF for IMU + odometry fusion
- Outputs fused odometry on /odometry/filtered topic
- TF tree is published for robot pose tracking
- Ready for SLAM integration (e.g., slam_toolbox, nav2)
