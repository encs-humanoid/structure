#!/usr/bin/env bash

#===================================================================
# Roscore Starter Installer
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
    echo "Usage: $0 <iface> <robot>"
    echo " where <iface> is wlan0, eth0, eth1, the network interface to associate with roscore"
    echo " where <robot> is the name of the robot in the load_param_<robot>.launch file"
    exit 1
}

if [ $# -ne 2 ]; then
    usage
fi

TARGET=/etc/init.d/roscore
UPPER_DIR=`pwd | sed 's/\/catkin_ws.*//g'`
SCRIPTS_DIR=$UPPER_DIR/catkin_ws/src/structure/scripts

IFACE=$1
ROBOT=$2
PARAM_FILE=$UPPER_DIR/catkin_ws/src/structure/ken/load_param_${ROBOT}.launch

if [ ! $IFACE = "wlan0" ] && [ ! $IFACE = "eth0" ] && [ ! $IFACE = "eth1" ]; then
    usage
fi

if [ ! -f $PARAM_FILE ]; then
    echo $PARAM_FILE "not found"
    usage
fi

# Create copy of roscore template
cp $SCRIPTS_DIR/roscore.template $SCRIPTS_DIR/roscore.tmp
# Inject the interface and robot parameters
sed -e "s/IFACE=.*/IFACE=${IFACE}/" -i $SCRIPTS_DIR/roscore.tmp
sed -e "s/ROBOT=.*/ROBOT=${ROBOT}/" -i $SCRIPTS_DIR/roscore.tmp
sed -e "s/RUSER=.*/RUSER=${USER}/" -i $SCRIPTS_DIR/roscore.tmp
# Copy to the target location and set permissions
sudo cp -f $SCRIPTS_DIR/roscore.tmp $TARGET
sudo chown root:root $TARGET
sudo chmod 755 $TARGET
rm -f $SCRIPTS_DIR/roscore.tmp

# # report success or failure
if [ ! -f $TARGET ]; then
    echo
    echo "Install failed. $TARGET not in place"
    echo "Exiting."
    echo
    exit 1
fi

# Configure roscore to start when the system boots
sudo update-rc.d roscore defaults 19

echo
echo "Install completed."
echo
echo "To start roscore now, run:"
echo "  sudo service roscore start"
echo

