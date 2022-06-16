#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup gazebo environment
source "/usr/share/gazebo/setup.sh"

exec "$@"
