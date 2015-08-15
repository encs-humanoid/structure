#!/usr/bin/env bash

while [ 1 ]; do

  echo "roscore_starter.ubuntu.sh:  Wait for network..."
  if [ $(ip add sh dev eth1 | grep inet | wc -l) -ne 0 ]; then
     break
  fi

  sleep 1

done

SCRIPT_DIR=/home/human/catkin_ws/src/structure/scripts

#source /opt/ros/indigo/setup.bash
source /home/human/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.8.100:11311/
export ROS_IP=192.168.8.100
#export ROS_IP=`/bin/hostname --all-ip-addresses | /usr/bin/cut -d\  -f1`

# Installer script should set next line to path of boot_node.
cd $SCRIPT_DIR
date >> roscore.log
date >> roscore.err
echo $PATH >> roscore.log
echo $PYTHONPATH >> roscore.log
roscore 1>>roscore.log 2>>roscore.err </dev/null &

# load the launch files into the parameter server
sleep 2
roslaunch structure load_param.launch

#espeak -v english-us -s 140 -p 80 -g 2 "Hi..." --stdout | aplay
#espeak -v english-us -s 140 -p 80 -g 2 "core initialized.... power on remaining systems." --stdout | aplay

rosrun speech-and-hearing speak_node.py &

rostopic pub /say std_msgs/String -1 "Hi"
