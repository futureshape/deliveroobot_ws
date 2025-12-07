#!/bin/bash

# Teleop YB_Car robot using keyboard

echo "Starting keyboard teleop for YB_Car..."
echo ""
echo "Controls:"
echo "  i - Forward"
echo "  k - Stop"
echo "  j - Turn left"
echo "  l - Turn right"
echo "  u/o - Forward + turn"
echo "  m/, - Backward + turn"
echo "  q/z - Increase/decrease speed"
echo ""
echo "Press Ctrl+C to stop"
echo ""

cd "$(dirname "$0")"
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel
