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
        self.actions = ["reset", "stop", "shutdown"]
        self.launch_process = None
        rospy.Subscriber('boot', String, self.on_boot_topic)
        rospy.init_node('boot')


    def on_boot_topic(self, msg):
        command = msg.data
        if command in self.actions:
            rospy.loginfo(rospy.get_caller_id() + ": Received command " + command)
            eval("self." + command + "()")
        else:
            rospy.loginfo(rospy.get_caller_id() + ": UNRECOGNIZED MESSAGE: " + msg.data)


    def nodes_are_up(self):
        return self.launch_process != None and os.path.exists("/proc/" + str(self.launch_process))


    # implement roslaunch boot.launch
    def launch_nodes(self):
        # FIXME : avoid file not found errors
        self.launch = Popen(["/opt/ros/hydro/bin/roslaunch", "./boot.launch"])
        self.launch_process = self.launch.pid
        rospy.loginfo(rospy.get_caller_id() + ": Child PID: " + str(self.launch_process))
        self.launch.communicate()


    def reset(self):
        rospy.loginfo(rospy.get_caller_id() + ": Resetting nodes")
        if self.nodes_are_up():
            rospy.loginfo(rospy.get_caller_id() + ": Found nodes up. Stopping...")
            self.stop()
        for nic in ni.interfaces():
            for iface in ni.ifaddresses(nic)[17]:
                req = "/role/boot_node_" + iface['addr'].replace(":", "")
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


    def stop(self):
        if self.nodes_are_up():
            rospy.loginfo(rospy.get_caller_id() +
                          ": Stopping node process: " + str(self.launch_process))
            os.kill(self.launch_process, signal.SIGQUIT)
            self.launch_process = None
        else:
            rospy.loginfo(rospy.get_caller_id() + ": Did not find running nodes")

    def shutdown(self):
        self.stop()
        os.system("sudo poweroff")


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        boot_node = BootNode()
        boot_node.run()
    except rospy.ROSInterruptException: pass
