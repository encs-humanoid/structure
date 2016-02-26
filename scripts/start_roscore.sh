#!/usr/bin/env bash

usage() {
    echo "Usage: $0 <iface> <robot>"
    echo " where <iface> is wlan0, eth0, eth1, etc."
    echo " where <robot> is the name of the robot in the load_param_<robot>.launch file"
    exit 1
}

listcontains() {
    for word in $1; do
	[[ $word = $2 ]] && return 0
    done
    return 1
}

# verify number of parameters provided
if [ $# -ne 2 ]; then
    usage
fi

echo "#### number of parameters is okay"

# get a list of all the network interfaces available
ifaces=`/bin/netstat -i | /bin/egrep -v "Kernel|Iface" | /usr/bin/cut -d\  -f1`

echo "#### found interfaces: " ${ifaces}

# verify that the parameter to the script is a valid network interface
if listcontains "${ifaces}" "$1"; then
    IFACE=$1
else
    echo "Invalid network interface.  Must be one of: " ${ifaces}
    usage
fi

ROBOT=$2

echo "#### set IFACE=$1"
echo "#### set ROBOT=$2"

while [ 1 ]; do

  echo "$0:  Wait for network..."
  if [ $(ip add sh dev ${IFACE} | grep inet | wc -l) -ne 0 ]; then
     break
  fi

  sleep 1

done

echo "#### $IFACE is up"

source /home/${USER}/catkin_ws/devel/setup.bash

echo "#### sourced ROS setup"

export ROS_IP=`/sbin/ip addr sh dev ${IFACE} | /bin/grep inet | /usr/bin/xargs | /usr/bin/cut -d\  -f2 | /usr/bin/cut -d/ -f1`

echo "#### set ROS_IP=$ROS_IP"

export ROS_MASTER_URI=http://${ROS_IP}:11311/

echo "#### set ROS_MASTER_URI=$ROS_MASTER_URI"

SCRIPT_DIR=/home/${USER}/catkin_ws/src/structure/scripts

# Installer script should set next line to path of boot_node.
cd $SCRIPT_DIR

echo "#### current directory is " `pwd`

date >> roscore.log
date >> roscore.err
echo $PATH >> roscore.log
echo $PYTHONPATH >> roscore.log
env | grep ROS >> roscore.log

echo "#### starting roscore"

roscore 1>>roscore.log 2>>roscore.err </dev/null &
PID=$!
sleep 2
ROSCORE_PID=`pgrep $PID`

echo "#### started roscore process $PID"

echo $PID > /tmp/roscore.pid

## load the launch files into the parameter server
if [ $(find .. -name load_param_${ROBOT}.launch | wc -l) -ne 1 ]; then
    echo "Could not find load_param_${ROBOT}.launch.  Launch files not loaded to parameter server."
    exit 1
else
    roslaunch structure load_param_${ROBOT}.launch --wait
    rosparam list
fi

