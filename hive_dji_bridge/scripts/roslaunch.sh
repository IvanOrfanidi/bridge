#!/bin/bash

# execute from .deb package
. /opt/ros/$(ls /opt/ros)/setup.bash
# execute from source code
#. ${HOME}/catkin_ws/devel/setup.bash

# settings
. ${HOME}/.drone_setup

rosclean purge -y
export ROS_HOME=${HOME}/.ros
roslaunch hive_dji_bridge hive_dji_bridge.launch
