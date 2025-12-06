#!/bin/bash

# Launch RViz2 locally on the laptop display with YB_Car configuration

echo "Starting RViz2 with YB_Car configuration..."

# Ensure we're using the local display
export DISPLAY=:0

# Source the workspace
cd "$(dirname "$0")"
source install/setup.bash

# Launch RViz with the custom config
rviz2 -d install/yb_car_localization/share/yb_car_localization/config/yb_car_view.rviz
