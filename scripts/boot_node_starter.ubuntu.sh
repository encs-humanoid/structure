#!/usr/bin/env bash

SCRIPT_DIR=/home/human/catkin_ws/src/structure/scripts

#source /opt/ros/hydro/setup.bash
source /home/human/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.100:11311/
export ROS_IP=192.168.8.100
#export ROS_IP=`/bin/hostname --all-ip-addresses | /usr/bin/xargs`

# Installer script should set next line to path of boot_node.
cd $SCRIPT_DIR
$SCRIPT_DIR/boot_node.py --shutdown_delay 15 > $SCRIPT_DIR/boot_node.log 2>&1
