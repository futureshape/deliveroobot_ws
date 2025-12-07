#!/bin/bash
# Start RViz for SLAM visualization (run locally on your laptop)

echo "Starting RViz for SLAM mapping visualization..."
echo "This will show the map being built in real-time."
echo ""

export DISPLAY=:0
cd /home/alex/deliveroobot_ws
source install/setup.bash

# Connect to remote ROS2 nodes
export ROS_DOMAIN_ID=0

ros2 launch yb_car_localization localization.launch.py enable_slam:=true launch_rviz:=true
