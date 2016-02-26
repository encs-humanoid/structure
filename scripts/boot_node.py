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
import argparse
import atexit
import netifaces as ni
import os
import roslaunch
import rospy
import signal
import socket
from std_msgs.msg import String
from subprocess import Popen
import sys
import threading
import time

AF_PACKET = 17

class BootNode(object):
    def __init__(self):
        rospy.init_node('boot', anonymous=True)

        self.actions = {"reset": self.reset, "stop": self.stop, "shutdown": self.shutdown, "status": self.status}
        self.launch_process = None

	myargs = rospy.myargv(sys.argv)  # process ROS args and return the rest
	parser = argparse.ArgumentParser(description="Start and manage ROS nodes at boot time")
	parser.add_argument("--shutdown_delay", default=0, type=int, help="Number of seconds to delay before issuing the poweroff command")
	self.options = parser.parse_args(myargs[1:])

        rospy.Subscriber('boot', String, self.on_boot_topic)
	self.status_publisher = rospy.Publisher("say", String)


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


    def reset(self, applies_to=set()):
	if len(applies_to) == 0 or len(self.get_mac_addresses() & applies_to) > 0:
	    rospy.loginfo(rospy.get_caller_id() + ": Resetting nodes")
	    if self.nodes_are_up():
		rospy.loginfo(rospy.get_caller_id() + ": Found nodes up. Stopping...")
		self.stop(applies_to)
	    mac_addresses = self.get_mac_addresses()
	    if len(applies_to) > 0:
		mac_addresses &= applies_to
	    rospy.loginfo(rospy.get_caller_id() + ": MAC addresses " + str(mac_addresses))
	    num_retries = 10
	    num_tries = 0
	    for num_tries in range(num_retries):
		launched = False
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
			t = threading.Thread(target=self.launch_nodes)
			t.setDaemon(True)
			t.start()
			#self.launch_nodes()
			launched = True
			break
		    if not rospy.has_param(req):
			rospy.loginfo(rospy.get_caller_id() + ": Could not find params for " + req)
		if launched:
		    break
		else:
		    time.sleep(1)


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
		os.kill(self.launch_process, signal.SIGINT)
		self.launch_process = None
	    else:
		rospy.loginfo(rospy.get_caller_id() + ": Did not find running nodes")


    def shutdown(self, applies_to):
	if len(applies_to) == 0 or len(self.get_mac_addresses() & applies_to) > 0:
	    self.stop(applies_to)
	    # delay a bit to allow messages to propagate before powering down
	    # this is especially important on the master node where the poweroff
	    # may happen before the message reaches the other computers on the robot
	    time.sleep(self.options.shutdown_delay)
	    os.system("sudo poweroff")


    def status(self, applies_to):
	if len(applies_to) == 0 or len(self.get_mac_addresses() & applies_to) > 0:
	    running_status = "running" if self.nodes_are_up() else "not running"
	    self.status_publisher.publish(socket.gethostname() + " nodes are " + running_status + "...")


    def run(self):
	time.sleep(5)  # HACK give a chance for the system to finish starting up
	self.reset()  # automatically start nodes on startup
	self.status_publisher.publish(socket.gethostname() + " is up")
        rospy.spin()


    def on_exit(self):
	self.stop("")


if __name__ == "__main__":
    try:
	boot_node = BootNode()
	atexit.register(boot_node.on_exit)
	boot_node.run()
    except rospy.ROSInterruptException:
	pass

