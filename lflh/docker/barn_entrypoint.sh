#!/bin/bash
set -e

# setup ros environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /root/jackal_ws/devel/setup.bash
exec "$@"
