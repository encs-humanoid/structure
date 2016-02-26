#!/usr/bin/env bash

#===================================================================
# Boot Node Starter Installer
#
# Daniel McDonald
# 02/26/2016
#
# NOTES:
#
# Copyright 2016, IEEE ENCS Humanoid Robot Project
#===================================================================

# verify script is running under the catkin_ws directory
pwd | grep catkin_ws > /dev/null
if [ $? == 1 ]; then
    echo
    echo "Please run this installer from somewhere below the catkin_ws directory."
    echo "Exiting without installing."
    echo
    exit 1
fi

usage() {
    echo "Usage: $0 <iface> <ros_master_ip> <shutdown_delay_sec>"
    echo " where <iface> is wlan0, eth0, eth1, etc."
    echo " where <ros_master_ip> is the ip address of the ROS master"
    echo " where <shutdown_delay_sec> is the number of seconds to delay before powering off the computer"
    echo "                            when the boot_node receives a shutdown message"
    exit 1
}

listcontains() {
    for word in $1; do
	[[ $word = $2 ]] && return 0
    done
    return 1
}

if [ $# -ne 3 ]; then
    usage
fi

# get a list of all the network interfaces available
ifaces=`/bin/netstat -i | /bin/egrep -v "Kernel|Iface" | /usr/bin/cut -d\  -f1`

# verify that the parameter to the script is a valid network interface
if listcontains "${ifaces}" "$1"; then
    IFACE=$1
else
    echo "Invalid network interface.  Must be one of: " ${ifaces}
    usage
fi

if [ $USER = "root" ]; then
    echo "This script should not be run as root."
    echo "Run it from the user account which will run the boot node."
    exit 1
fi

ROS_MASTER_IP=$2
SHUTDOWN_DELAY=$3

TARGET=/etc/init.d/ros-boot-node
UPPER_DIR=`pwd | sed 's/\/catkin_ws.*//g'`
SCRIPTS_DIR=$UPPER_DIR/catkin_ws/src/structure/scripts

# Create copy of roscore template
cp $SCRIPTS_DIR/ros-boot-node.template $SCRIPTS_DIR/ros-boot-node.tmp
# Inject the ROS master IP, interface, and shutdown delay parameters
sed -e "s/ROS_MASTER_IP=.*/ROS_MASTER_IP=${ROS_MASTER_IP}/" -i $SCRIPTS_DIR/ros-boot-node.tmp
sed -e "s/IFACE=.*/IFACE=${IFACE}/" -i $SCRIPTS_DIR/ros-boot-node.tmp
sed -e "s/SHUTDOWN_DELAY=.*/SHUTDOWN_DELAY=${SHUTDOWN_DELAY}/" -i $SCRIPTS_DIR/ros-boot-node.tmp
sed -e "s/RUSER=.*/RUSER=${USER}/" -i $SCRIPTS_DIR/ros-boot-node.tmp
# Copy to the target location and set permissions
sudo cp -f $SCRIPTS_DIR/ros-boot-node.tmp $TARGET
sudo chown root:root $TARGET
sudo chmod 755 $TARGET
rm -f $SCRIPTS_DIR/ros-boot-node.tmp

# # report success or failure
if [ ! -f $TARGET ]; then
    echo
    echo "Install failed. $TARGET not in place"
    echo "Exiting."
    echo
    exit 1
fi

# Configure roscore to start when the system boots
sudo update-rc.d ros-boot-node defaults 20

echo
echo "Install completed."
echo
echo "To start ros-boot-node now, run:"
echo "  sudo service ros-boot-node start"
echo

