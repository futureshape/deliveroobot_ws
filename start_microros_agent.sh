#!/bin/bash

# Start MicroROS agent for YB_Car robot
# This connects to the robot via UDP on port 8090

echo "Starting MicroROS agent for YB_Car..."
echo "Waiting for robot connection on UDP port 8090..."

sudo docker run -it --rm \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  --privileged \
  --net=host \
  microros/micro-ros-agent:jazzy udp4 --port 8090 -v4
