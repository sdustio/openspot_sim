#!/bin/bash
set -e

# setup openspot environment
source "/openspot/install/setup.bash"

# setup gazebo environment
source "/usr/share/gazebo/setup.sh"

exec "$@"
