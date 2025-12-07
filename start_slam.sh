#!/bin/bash
# Start SLAM mapping with the YB_Car robot
# This will create a map of your environment as you drive around

echo "==================================================================="
echo "  YB_Car SLAM Mapping"
echo "==================================================================="
echo ""
echo "⚠️  IMPORTANT: STOP localization.launch.py if it's running!"
echo "   (SLAM and localization conflict - use Ctrl+C in that terminal)"
echo ""
echo "Starting SLAM Toolbox for mapping..."
echo ""
echo "REQUIRED before starting:"
echo "  1. MicroROS agent running (./start_microros_agent.sh)"
echo "  2. Robot powered on and connected"
echo "  3. Localization launch STOPPED (if it was running)"
echo ""
echo "After starting SLAM, you should:"
echo "  1. Open RViz to see the map (./start_slam_rviz.sh)"
echo "  2. Drive the robot around (./teleop_keyboard.sh)"
echo "  3. Save the map when done (./save_map.sh)"
echo ""
echo "==================================================================="
echo ""

cd /home/alex/deliveroobot_ws
source install/setup.bash
ros2 launch yb_car_localization localization.launch.py enable_slam:=true launch_rviz:=false
