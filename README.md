# YB_Car ROS2 Localization Package

ROS2 workspace for sensor fusion and pose estimation on the YB_Car robot using `robot_localization` EKF (Extended Kalman Filter).

## Overview

This package fuses IMU and odometry data from your YB_Car robot to provide accurate pose estimation for SLAM and navigation. The MicroROS agent creates a node that publishes sensor data which this package processes.

**Status**: ✅ Fully configured and tested with YB_Car hardware

## Features

- **Sensor Fusion**: Combines IMU (`/imu`) and raw odometry (`/odom_raw`) using Extended Kalman Filter
- **Filtered Output**: Publishes fused odometry on `/odometry/filtered` topic
- **TF Tree**: Maintains complete robot transforms for localization
- **SLAM Ready**: Optimized configuration for SLAM toolbox and Nav2 integration
- **Visualization**: RViz2 configuration for real-time monitoring
- **Hardware Tested**: Configured for YB_Car with manufacturer's URDF and sensor setup

## Topics

### Subscribed (from YB_Car_Node via MicroROS)
- `/imu` (sensor_msgs/Imu) - IMU sensor data (frame: `imu_frame`)
- `/odom_raw` (nav_msgs/Odometry) - Raw wheel odometry (frame: `odom_frame` → `base_footprint`)
- `/scan` (sensor_msgs/LaserScan) - Laser scan data (frame: `laser_frame`, visualized, not fused)
- `/battery` (std_msgs/UInt16) - Battery status (not used in fusion)

### Published (by this package)
- `/odometry/filtered` (nav_msgs/Odometry) - Fused odometry from EKF
- `/tf` - Transform tree for robot localization

### Robot Frame Structure
```
odom_frame
  └── base_footprint
      └── base_link
          ├── imu_Link → imu_frame
          ├── radar_Link → laser_frame, laser_link
          ├── zq_Link (left front wheel)
          ├── yq_Link (right front wheel)
          ├── yh_Link (right rear wheel)
          └── zh_Link (left rear wheel)
```

## Prerequisites

### System Requirements
- ROS2 (Humble/Iron/Jazzy recommended)
- Ubuntu 22.04 or later (for ROS2 Humble)

### Dependencies
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-robot-localization \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher \
                 ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-rviz2
```

## Installation

1. **Navigate to your workspace:**
   ```bash
   cd ~/deliveroobot_ws
   ```

2. **Build the package:**
   ```bash
   colcon build --packages-select yb_car_localization
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Robot Description (URDF)

✅ **Already configured!** The package includes the manufacturer's URDF (`MicroROS.urdf`) with the following:

- **base_footprint** - Ground reference frame (odometry child frame)
- **base_link** - Main robot body (raised 2.125cm above base_footprint)
- **imu_Link** - IMU sensor at position (-0.003, -0.003, 0.032) meters from base_link
- **imu_frame** - Frame matching the actual IMU data topic
- **radar_Link** - LiDAR sensor at position (0, 0, 0.079) meters from base_link
- **laser_frame** - Frame matching the actual laser scan topic
- **laser_link** - Standard SLAM-compatible laser frame alias
- **Four wheel links** - Complete 4WD kinematics (zq, yq, yh, zh)
- **Servo joints** - Two revolute joints for accessories

The URDF is located at `urdf/yb_car.urdf.xacro` and includes the original `MicroROS.urdf` with additional frame definitions for ROS2 compatibility.

## Usage

### 1. Start MicroROS Agent
Use the provided script to start the MicroROS agent:
```bash
cd ~/deliveroobot_ws
./start_microros_agent.sh
```

This runs the agent via Docker on UDP port 8090. To stop it:
```bash
./stop_microros_agent.sh
```

**Manual command (if needed):**
```bash
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm \
  --privileged --net=host \
  microros/micro-ros-agent:humble udp4 --port 8090 -v4
```

### 2. Verify Robot Topics
Check that data is being published:
```bash
ros2 topic list
ros2 topic hz /imu        # Should be ~25 Hz
ros2 topic hz /odom_raw   # Should be ~10 Hz
ros2 topic hz /scan       # Laser scan rate
```

### 3. Launch Sensor Fusion Only
To run just the EKF sensor fusion node:
```bash
ros2 launch yb_car_localization sensor_fusion.launch.py
```

### 4. Launch Complete Localization (Recommended)
To run sensor fusion with robot description and RViz visualization:

**For remote SSH (without RViz):**
```bash
ros2 launch yb_car_localization localization.launch.py launch_rviz:=false
```

**For local display (with RViz):**
```bash
ros2 launch yb_car_localization localization.launch.py
```

**Or use the RViz script on the laptop display:**
```bash
# In SSH terminal - run localization without RViz
ros2 launch yb_car_localization localization.launch.py launch_rviz:=false

# On laptop display - run RViz separately
cd ~/deliveroobot_ws
./start_rviz.sh
```

This launches:
- Robot State Publisher (publishes TF from URDF)
- EKF Node (sensor fusion)
- RViz2 (visualization - optional)

### 5. Monitor Filtered Output
Check the fused odometry output:
```bash
ros2 topic echo /odometry/filtered
```

## Visualization

### RViz2 (Local Display)
For local visualization on the laptop screen:
```bash
./start_rviz.sh
```

The RViz configuration includes:
- 3D robot model with all links
- LaserScan visualization
- Filtered odometry pose with covariance
- TF tree display
- Grid reference

### Foxglove Studio (Remote/Web-Based)
Perfect for remote development over SSH:

1. **Install Foxglove Bridge:**
   ```bash
   sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
   ```

2. **Run the bridge:**
   ```bash
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml
   ```

3. **Open in browser:**
   - Go to https://app.foxglove.dev
   - Connect to `ws://YOUR_LAPTOP_IP:8765`
   - Add 3D, LaserScan, and Plot panels

### TF Tree Visualization
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

## Configuration

### EKF Parameters
The EKF configuration is in `config/ekf.yaml`. Key parameters:

- **Frame Configuration**:
  - `odom_frame: odom_frame` - Matches robot's odometry parent frame
  - `base_link_frame: base_footprint` - Matches robot's odometry child frame
  - `world_frame: odom_frame` - Reference frame for fusion
- **`two_d_mode: true`** - Set to `false` for 3D SLAM
- **`odom0_config`** - Uses x/y velocities and yaw rate from wheel odometry
- **`imu0_config`** - Uses orientation, angular velocities, and linear accelerations from IMU
- **`imu0_remove_gravitational_acceleration: true`** - Removes gravity from acceleration data
- **Covariance Handling**:
  - `imu0_reject_on_bad_covariance: false` - Accepts IMU data with zero covariance
  - `odom0_reject_on_bad_covariance: false` - For robustness with various sensors

### Tuning Tips

1. **If pose drifts**: Increase process noise covariance
2. **If response is sluggish**: Decrease process noise, increase sensor trust
3. **For 3D SLAM**: Set `two_d_mode: false` in `ekf.yaml`
4. **IMU orientation**: Ensure IMU frame matches your URDF transform

## Integration with SLAM

Once you have filtered odometry, you can integrate SLAM:

### Using SLAM Toolbox
```bash
sudo apt install ros-${ROS_DISTRO}-slam-toolbox

# Run SLAM
ros2 launch slam_toolbox online_async_launch.py
```

### Using Nav2
```bash
sudo apt install ros-${ROS_DISTRO}-navigation2

# Configure Nav2 to use /odometry/filtered
```

## Troubleshooting

### No Filtered Output
- ✅ **Fixed**: EKF now accepts data with zero covariance
- Check that `/imu` and `/odom_raw` are publishing: `ros2 topic hz /imu`
- Check EKF node logs: `ros2 node info /ekf_filter_node`
- Verify frames match: `ros2 topic echo /imu --once | grep frame_id`

### TF Errors
- ✅ **Fixed**: All required frames configured in URDF
- Ensure URDF has all required links and joints
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify timestamps in sensor messages are correct

### Missing Transform Errors
- ✅ **Fixed**: Frame names now match robot output (`imu_frame`, `laser_frame`, `base_footprint`)
- The robot publishes:
  - IMU in `imu_frame`
  - Odometry with `odom_frame` → `base_footprint`
  - Laser in `laser_frame`

### RViz Display Issues (SSH/Remote)
- ✅ **Solutions provided**:
  - Use `launch_rviz:=false` when launching remotely
  - Run `./start_rviz.sh` directly on laptop
  - Use Foxglove Studio for web-based visualization
- Set `DISPLAY=:0` if running on local display

### RViz LaserScan Queue Full
- ✅ **Fixed**: Increased filter size from 10 to 100 in RViz config
- If still occurring, increase to 200-500 in `config/yb_car_view.rviz`

### IMU Data Issues
- ✅ **Fixed**: EKF configured to handle zero covariance from YB_Car IMU
- IMU calibration should be done on robot firmware if needed
- Gravity removal is enabled in EKF config

### Odometry Drift
- Tune EKF covariance matrices in `ekf.yaml`
- Check wheel encoder accuracy
- Consider adding additional sensors (GPS, visual odometry)

## Package Structure

```
yb_car_localization/
├── config/
│   ├── ekf.yaml                     # EKF configuration (frame-matched for YB_Car)
│   ├── yb_car_view.rviz             # RViz config for localization
│   ├── slam_toolbox_config.yaml     # SLAM Toolbox parameters
│   └── slam_view.rviz               # RViz config for SLAM
├── launch/
│   ├── sensor_fusion.launch.py      # EKF only
│   ├── localization.launch.py       # Complete localization system
│   └── slam.launch.py               # SLAM mapping system
├── urdf/
│   ├── yb_car.urdf.xacro     # Main URDF wrapper
│   └── MicroROS.urdf         # Manufacturer's robot description
├── meshes/                   # STL files for visualization
├── CMakeLists.txt
├── package.xml
└── README.md

Workspace root scripts:
├── start_microros_agent.sh   # Start MicroROS Docker agent
├── stop_microros_agent.sh    # Stop MicroROS agent
├── start_rviz.sh             # Launch RViz for localization
├── start_slam.sh             # Start SLAM mapping
├── start_slam_rviz.sh        # Launch RViz for SLAM
├── save_map.sh               # Save current SLAM map
└── teleop_keyboard.sh        # Keyboard teleoperation

Maps directory:
└── maps/                     # Saved SLAM maps stored here
```

## SLAM Mapping

### Quick Start - Build Your First Map

1. **Start the MicroROS agent** (connects to your robot):
```bash
./start_microros_agent.sh
```

2. **Start SLAM Toolbox** (in a new terminal):
```bash
./start_slam.sh
```

3. **Open RViz for visualization** (on your laptop - new terminal):
```bash
./start_slam_rviz.sh
```

4. **Drive the robot around** (in a new terminal):
```bash
./teleop_keyboard.sh
```

Use keyboard to drive:
- `i` = forward
- `k` = stop  
- `j` = turn left
- `l` = turn right
- `u`/`o` = arc left/right
- `m`/`,` = backward
- `q`/`z` = increase/decrease speed

5. **Watch the map build** in RViz as you drive around your environment

6. **Save the map** when done:
```bash
./save_map.sh my_room_map
```

### Understanding SLAM

SLAM (Simultaneous Localization and Mapping) allows your robot to:
- Build a map of an unknown environment
- Track its position within that map
- Close loops when revisiting known areas

**What you'll see:**
- **Map** (gray grid): Occupancy grid showing free space (white), obstacles (black), unknown (gray)
- **Laser scans** (colored dots): Current LiDAR readings
- **Robot pose**: Position and orientation in the map
- **TF tree**: Map → odom_frame → base_footprint transforms

### SLAM Configuration

The SLAM setup is configured in `config/slam_toolbox_config.yaml`:

**Key Parameters:**
- `minimum_travel_distance: 0.2` - Process scans every 20cm movement
- `minimum_travel_heading: 0.2` - Process scans every 0.2 rad rotation
- `resolution: 0.05` - Map resolution 5cm per pixel
- `max_laser_range: 12.0` - Use laser readings up to 12m
- `do_loop_closing: true` - Detect and correct drift when revisiting areas

**Frames:**
- `map_frame: map` - Global map coordinate frame
- `odom_frame: odom_frame` - Odometry frame (matches YB_Car output)
- `base_frame: base_footprint` - Robot base frame

### Manual SLAM Launch

If you want more control over the launch:

```bash
# Launch just SLAM (no RViz)
ros2 launch yb_car_localization slam.launch.py launch_rviz:=false

# Launch with RViz included
ros2 launch yb_car_localization slam.launch.py launch_rviz:=true
```

### Saving Maps

**Automatic save with script:**
```bash
./save_map.sh my_map_name
```

**Manual save using ROS services:**
```bash
# Save as SLAM pose graph (for later localization)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/alex/deliveroobot_ws/maps/my_map'}"

# Save as image + YAML (for Nav2)
ros2 run nav2_map_server map_saver_cli -f /home/alex/deliveroobot_ws/maps/my_map
```

This creates:
- `my_map.pgm` - Map image (grayscale)
- `my_map.yaml` - Map metadata (resolution, origin, thresholds)
- `my_map.posegraph` - SLAM pose graph
- `my_map.data` - SLAM internal data

### Tips for Good Maps

1. **Drive slowly** - Give SLAM time to process scans
2. **Close loops** - Return to starting point to reduce drift
3. **Good lighting** - Ensure LiDAR has clear line of sight
4. **Overlap coverage** - Drive paths that overlap for better loop closure
5. **Avoid dynamic objects** - People walking, moving furniture creates artifacts
6. **Feature-rich environment** - Corners and walls work better than open spaces

### Troubleshooting SLAM

**Map not building:**
```bash
# Check SLAM is receiving scans
ros2 topic hz /scan

# Check SLAM is publishing map
ros2 topic hz /map

# View SLAM logs
ros2 node info /slam_toolbox
```

**Robot pose jumping:**
- Too fast movement - slow down
- Poor loop closure - improve overlap
- Adjust `loop_match_minimum_response_fine` in config

**Map quality issues:**
- Increase `scan_buffer_size` for smoother maps
- Decrease `minimum_travel_distance` for more detail
- Tune `correlation_search_space_*` parameters

## Next Steps

1. ✅ **Sensor Fusion**: Configured and working (`/odometry/filtered`)
2. ✅ **SLAM Mapping**: Ready to build maps with SLAM Toolbox
3. **Localization**: Use saved maps for robot localization (AMCL or SLAM localization mode)
4. **Navigation**: Set up Nav2 for autonomous path planning and obstacle avoidance
5. **Advanced**: Multi-floor mapping, semantic mapping, exploration

## Additional Resources

- [robot_localization Documentation](http://docs.ros.org/en/latest/p/robot_localization/)
- [ROS2 TF2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Foxglove Studio](https://foxglove.dev/)

## Hardware Notes

### YB_Car Robot Specifications
- **Manufacturer**: Yahboom (微控小车 MicroROS)
- **Drive**: 4WD (four independent wheels)
- **Sensors**: 
  - IMU at (-3mm, -3mm, 32mm) from base_link
  - LiDAR/Radar at (0, 0, 79mm) from base_link
  - Battery monitor
- **Connectivity**: MicroROS agent via UDP (default port 8090)
- **ROS2 Distro**: Tested with Humble/Jazzy

### Known Issues & Solutions
- **IMU Zero Covariance**: YB_Car publishes IMU with zero covariance - EKF configured to accept this
- **Frame Naming**: Robot uses non-standard frame names (`imu_frame`, `laser_frame`, `odom_frame`) - URDF aliases created
- **Remote Display**: Use Foxglove or local RViz script for SSH development

## License

MIT

## Author

Alex
