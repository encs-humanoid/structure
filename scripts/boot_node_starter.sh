#!/usr/bin/env bash

SCRIPT_DIR=/home/pi/catkin_ws/src/structure/scripts

source /opt/ros/groovy/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.100:11311/
export ROS_IP=`hostname --all-ip-addresses | xargs`

# Installer script should set next line to path of boot_node.
cd $SCRIPT_DIR
$SCRIPT_DIR/boot_node.py > $SCRIPT_DIR/boot_node.log 2>&1
