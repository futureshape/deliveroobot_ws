#!/bin/bash

# Stop MicroROS agent for YB_Car robot

echo "Stopping MicroROS agent..."

# Find and kill the docker container running micro-ros-agent (any version)
CONTAINER_ID=$(sudo docker ps | grep "microros/micro-ros-agent" | awk '{print $1}')

if [ -z "$CONTAINER_ID" ]; then
    echo "No MicroROS agent container found running."
    exit 0
fi

echo "Found container: $CONTAINER_ID"
sudo docker kill $CONTAINER_ID

echo "MicroROS agent stopped."
