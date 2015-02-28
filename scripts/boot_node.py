#!/usr/bin/env python

#===================================================================
# This is the Boot Node.
#
# Subscribes To:
# - /boot
#
# Publishes To:
# - /boot_info
# - /launch_info_<this_mac_address>
#
# Invoked at boot time by upstart init file, installed as
# /etc/init/ros-boot-node.conf, this program should be run at startup
# on every computer in the robot.
#
# TODO:
# - pub every 10 secs to /boot_info, status + ip + mac
# - stream debugging output messages from roslaunch, to /launch_info_<mac_address>
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import print_function
import rospy
from std_msgs.msg import String
import os
import netifaces as ni
import roslaunch
import signal
from subprocess import Popen

AF_PACKET = 17

class BootNode(object):
    def __init__(self):
        self.actions = {"reset": self.reset, "stop": self.stop, "shutdown": self.shutdown}
        self.launch_process = None
        rospy.Subscriber('boot', String, self.on_boot_topic)
        rospy.init_node('boot', anonymous=True)


    def on_boot_topic(self, msg):
	params = msg.data.split()
	if len(params) > 0:
	    command = params[0]
	    applies_to = set()
	    if len(params) > 1:
		applies_to = set(params[1:])
	    if command in self.actions.keys():
		rospy.loginfo(rospy.get_caller_id() + ": Received command " + command)
		self.actions[command](applies_to)
	    else:
		rospy.loginfo(rospy.get_caller_id() + ": UNRECOGNIZED COMMAND: " + command)
        else:
            rospy.loginfo(rospy.get_caller_id() + ": UNRECOGNIZED MESSAGE: " + msg.data)


    def nodes_are_up(self):
        return self.launch_process != None and os.path.exists("/proc/" + str(self.launch_process))


    # implement roslaunch boot.launch
    def launch_nodes(self):
        # FIXME : avoid file not found errors
        self.launch = Popen(["roslaunch", "./boot.launch"])
        self.launch_process = self.launch.pid
        rospy.loginfo(rospy.get_caller_id() + ": Child PID: " + str(self.launch_process))
        self.launch.communicate()


    def reset(self, applies_to):
	if len(applies_to) == 0 or len(self.get_mac_addresses() & applies_to) > 0:
	    rospy.loginfo(rospy.get_caller_id() + ": Resetting nodes")
	    if self.nodes_are_up():
		rospy.loginfo(rospy.get_caller_id() + ": Found nodes up. Stopping...")
		self.stop()
	    mac_addresses = self.get_mac_addresses()
	    if len(applies_to) > 0:
		mac_addresses &= applies_to
	    rospy.loginfo(rospy.get_caller_id() + ": MAC addresses " + str(mac_addresses))
	    for mac in mac_addresses:
		req = "/role/boot_node_" + mac
		if rospy.has_param(req):
		    rospy.loginfo(rospy.get_caller_id() + ": Request for " + req)
		    launch_file_contents = rospy.get_param(req)
		    rospy.loginfo(rospy.get_caller_id() +
				  ": Writing contents to boot.launch file")
		    with open("boot.launch", "w") as f:
			f.write(launch_file_contents)
		    rospy.loginfo(rospy.get_caller_id() +
				  ": Launching boot.launch")
		    self.launch_nodes()
		    break
		if not rospy.has_param(req):
		     rospy.loginfo(rospy.get_caller_id() + ": Could not find params for " + req)


    def get_mac_addresses(self):
	mac_addresses = set()
        for nic in ni.interfaces():
            for iface in ni.ifaddresses(nic)[17]:
                mac = iface['addr'].replace(":", "")
		mac_addresses.add(mac)
	return mac_addresses


    def stop(self, applies_to):
	if len(applies_to) == 0 or len(self.get_mac_addresses() & applies_to) > 0:
	    if self.nodes_are_up():
		rospy.loginfo(rospy.get_caller_id() +
			      ": Stopping node process: " + str(self.launch_process))
		os.kill(self.launch_process, signal.SIGQUIT)
		self.launch_process = None
	    else:
		rospy.loginfo(rospy.get_caller_id() + ": Did not find running nodes")


    def shutdown(self, applies_to):
	if len(applies_to) == 0 or len(self.get_mac_addresses() & applies_to) > 0:
	    self.stop()
	    os.system("sudo poweroff")


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        boot_node = BootNode()
        boot_node.run()
    except rospy.ROSInterruptException: pass
