#!/usr/bin/env bash

usage() {
    echo "Usage: $0 <iface> <ros_master_ip> <shutdown_delay_sec>"
    exit 1
}

# verify number of parameters provided
if [ $# -ne 3 ]; then
    usage
fi

IFACE=$1
ROS_MASTER_IP=$2
SHUTDOWN_DELAY=$3

echo "#### set IFACE=$1"
echo "#### set ROS_MASTER_IP=$2"
echo "#### set SHUTDOWN_DELAY=$3"

while [ 1 ]; do

  echo "$0:  Wait for network..."
  if [ $(ip add sh dev ${IFACE} | grep inet | wc -l) -ne 0 ]; then
     break
  fi

  sleep 1

done

# Set up the python virtual environment
source /home/human/venv/bin/activate
# Set up the ROS environment
source /home/${USER}/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://${ROS_MASTER_IP}:11311/
#export ROS_IP=`/sbin/ip addr sh dev ${IFACE} | /bin/grep inet | /usr/bin/xargs | /usr/bin/cut -d\  -f2 | /usr/bin/cut -d/ -f1`

# Start the boot_node
SCRIPT_DIR=/home/${USER}/catkin_ws/src/structure/scripts
cd $SCRIPT_DIR
$SCRIPT_DIR/boot_node.py --shutdown_delay $SHUTDOWN_DELAY > $SCRIPT_DIR/boot_node.log 2>&1 </dev/null &
PID=$!
echo $PID > /tmp/ros-boot-node.pid

