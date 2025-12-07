# SLAM Quick Start Guide

## Building Your First Map

### 1. Setup (one-time)
Make sure you have:
- ✅ Built the workspace: `colcon build --symlink-install`
- ✅ Sourced the workspace: `source install/setup.bash`
- ✅ Robot powered on and connected to network
- ✅ Made scripts executable: `chmod +x *.sh`

### 2. Start Mapping (in order)

**FIRST: Stop localization if running**
If you have `localization.launch.py` running, stop it with Ctrl+C.
SLAM and localization conflict because both publish transforms.

**Terminal 1 - MicroROS Agent:**
```bash
./start_microros_agent.sh
```
Wait for: "UDP agent initialization... OK"

**Terminal 2 - SLAM Toolbox:**
```bash
./start_slam.sh
```
Wait for: "[slam_toolbox]: Message Filter subscribing to topics"

**Terminal 3 - RViz Visualization (on laptop):**
```bash
./start_slam_rviz.sh
```
You should see the robot model and laser scans

**Terminal 4 - Drive the Robot:**
```bash
./teleop_keyboard.sh
```

### 3. Build the Map

Drive slowly around your environment:
- Use `i` (forward), `j` (left), `l` (right), `k` (stop)
- Watch the map appear in RViz (white = free, black = obstacles, gray = unknown)
- Try to return to your starting point (loop closure)
- Cover all areas you want mapped

### 4. Save the Map

When your map looks good:
```bash
./save_map.sh my_room
```

This saves to: `/home/alex/deliveroobot_ws/maps/my_room.*`

## Files Created

After mapping, you'll have:
- `my_room.pgm` - Map image (for Nav2)
- `my_room.yaml` - Map metadata (origin, resolution)
- `my_room.posegraph` - SLAM graph (for localization)
- `my_room.data` - SLAM internal data

## Troubleshooting

### No map appearing in RViz
```bash
# Check SLAM is receiving scans
ros2 topic hz /scan

# Check map is being published
ros2 topic hz /map

# Verify SLAM node is running
ros2 node list | grep slam_toolbox
```

### Robot pose is wrong
- Make sure localization is NOT running (conflicts with SLAM)
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify odometry: `ros2 topic echo /odometry/filtered`

### Map has artifacts
- Drive more slowly
- Ensure good laser scan coverage
- Remove dynamic objects (people, pets)
- Try adjusting SLAM parameters in `config/slam_toolbox_config.yaml`

## Tips for Best Results

1. **Go slow** - SLAM needs time to process
2. **Good overlap** - Drive overlapping paths
3. **Close loops** - Return to starting area
4. **Feature-rich areas** - Corners and walls work better than empty rooms
5. **Static environment** - Avoid moving objects
6. **Smooth motion** - Avoid sudden stops/starts

## What's Next?

After mapping:
1. **Localization** - Use your map to localize the robot (AMCL or SLAM localization mode)
2. **Navigation** - Set up Nav2 for autonomous navigation
3. **Multiple maps** - Create maps of different rooms/floors
4. **Map editing** - Clean up maps with GIMP or image editors

## Key ROS2 Topics

While SLAM is running:
- `/scan` - Laser scan input
- `/map` - Occupancy grid map being built
- `/odometry/filtered` - Fused odometry for SLAM
- `/tf` - Transforms (map → odom_frame → base_footprint)
- `/slam_toolbox/graph_visualization` - Pose graph for debugging

## Important Notes

- **Don't run localization.launch.py while mapping** - It will conflict with SLAM
- **Maps are saved in `maps/` directory** - Check there after saving
- **SLAM uses filtered odometry** - Make sure sensor fusion is working first
- **First map takes practice** - Don't worry if it's not perfect!
