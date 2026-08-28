#!/bin/bash
set -e

echo "Starting ROS 2 container as user: $(whoami) (UID=$(id -u))"

# Source ROS2 & rosbridge
source /opt/ros/foxy/install/setup.bash
source /ros2_lib/install/setup.bash

# Build mounted workspace
echo "Building workspace..."
cd /ros2_ws

# Only build if src exists (prevents errors on first run with empty volume)
if [ -d "src" ] && [ "$(ls -A src 2>/dev/null)" ]; then
    colcon build --symlink-install
    echo "Workspace build complete"
else
    echo "No packages found in src/ — skipping build"
fi

# Source workspace if it was successfully built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

echo "ROS_DISTRO: $ROS_DISTRO"
echo "Workspace:  /ros2_ws"
echo "ROS 2 environment ready"

exec "$@"
