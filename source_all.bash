#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

# CATKIN_SHELL=bash

# source setup.sh from same directory as this file
# _CATKIN_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
# . "$_CATKIN_SETUP_DIR/setup.sh"

ORBO_DRIVERS_WS=drivers_ws
ORBO_ROBOT_WS=orbo_ws
ORBO_SIMULATION_WS=orbo_simulation_ws
VREP_ROS_INTERFACE_WS=vrep_ros_interface_ws

source $ORBO_DRIVERS_WS/devel/setup.bash --extend
source $ORBO_ROBOT_WS/devel/setup.bash --extend
source $ORBO_SIMULATION_WS/devel/setup.bash --extend
source $VREP_ROS_INTERFACE_WS/devel/setup.bash --extend
