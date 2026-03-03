#!/usr/bin/env bash
set -e

# Source ROS2 setup
# shellcheck source=opt/ros/humble/setup.bash
# shellcheck disable=SC1091
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "Warning: /opt/ros/${ROS_DISTRO}/setup.bash not found."
fi

# Source workspace setup, if it exists
# shellcheck disable=SC1091
if [ -f "/carla/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/carla/install/setup.bash"
else
    echo "Warning: /carla/install/setup.bash not found. Skipping."
fi

# Export environment variables
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Execute the provided command
exec "$@"
