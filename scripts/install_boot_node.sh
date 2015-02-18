#!/usr/bin/env bash

#===================================================================
# Boot Node Installer
#
# Mark Whelan <m@townunderground.net>
# 02/17/2015
#
# NOTES:
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

# If run from within catkin_ws, we can set the boot_node.conf file
# to refer correctly to the executable.

pwd | grep catkin_ws > /dev/null
if [ $? == 1 ]; then
    echo
    echo "Please run this installer from somewhere below the catkin_ws directory."
    echo "Exiting without installing."
    echo
    exit 1
fi

TARGET=/etc/init/ros-boot-node.conf
UPPER_DIR=`pwd | sed 's/\/catkin_ws.*//g'`
BOOT_NODE_DIR=$UPPER_DIR/catkin_ws/src/structure/scripts

# determine if upstart is available on this machine
dpkg --get-selections | grep ^upstart > /dev/null
if [ $? == 1 ];then
    sudo apt-get install upstart -y
fi

# Create upstart init config
cp $BOOT_NODE_DIR/ros-boot-node.conf.template $BOOT_NODE_DIR/tmp.conf
# add exec line appropriate for this machine
echo "exec bash -c '$BOOT_NODE_DIR/boot_node_starter.sh'" >> $BOOT_NODE_DIR/tmp.conf
sudo cp -f $BOOT_NODE_DIR/tmp.conf $TARGET
sudo chown root:root $TARGET
sudo chmod 644 $TARGET
rm -f $BOOT_NODE_DIR/tmp.conf

# Create the starter script
cp -f $BOOT_NODE_DIR/boot_node_starter.sh.template $BOOT_NODE_DIR/boot_node_starter.sh
chmod +x $BOOT_NODE_DIR/boot_node_starter.sh
# Adjust the boot_node_starter.sh script to have correct path
echo "python $BOOT_NODE_DIR/boot_node.py" >> $BOOT_NODE_DIR/boot_node_starter.sh

# # report success or failure
if [ ! -f $TARGET ]; then
    echo
    echo "Install failed. $TARGET not in place"
    echo "Exiting."
    echo
    exit 1
fi

echo
echo "Install completed."
echo
echo "To start the boot node now, run:"
echo "  sudo service ros-boot-node start"
echo
