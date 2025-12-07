#!/bin/bash
# Save the current SLAM map to files

echo "==================================================================="
echo "  Save SLAM Map"
echo "==================================================================="
echo ""

# Default map name with timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_NAME="${1:-my_map_$TIMESTAMP}"
MAP_DIR="/home/alex/deliveroobot_ws/maps"

echo "Saving map as: $MAP_NAME"
echo "Location: $MAP_DIR"
echo ""

cd /home/alex/deliveroobot_ws
source install/setup.bash

# Call the SLAM Toolbox service to serialize (save) the map
echo "Calling SLAM Toolbox to save the map..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$MAP_DIR/$MAP_NAME'}"

echo ""
echo "Map saved! The following files were created:"
echo "  - $MAP_DIR/$MAP_NAME.posegraph"
echo "  - $MAP_DIR/$MAP_NAME.data"
echo ""
echo "You can also save as image and metadata using map_server:"
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME"

echo ""
echo "==================================================================="
echo "Map files created:"
echo "  - $MAP_NAME.pgm        (map image)"
echo "  - $MAP_NAME.yaml       (map metadata)"
echo "  - $MAP_NAME.posegraph  (SLAM graph)"
echo "  - $MAP_NAME.data       (SLAM data)"
echo "==================================================================="
