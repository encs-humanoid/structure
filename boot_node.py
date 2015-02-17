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
# /etc/init/ros_boot.conf, this program should be run at startup
# on every computer in the robot.
#
# TODO:
# - launch boot.launch and save pid
# - pub every 10 secs to /boot_info, status + ip + mac
# - stream debugging output messages from roslaunch, to /launch_info_<mac_address>
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import print_function
import rospy
import roslaunch
from std_msgs.msg import String
import os
import netifaces as ni
import roslaunch
#from subprocess import check_output # maybe

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
            rospy.loginfo(rospy.get_caller_id() + ": " + command)
            eval("self." + command + "()")
        else:
            rospy.loginfo(rospy.get_caller_id() + ": UNRECOGNIZED MESSAGE: " + msg.data)


    def nodes_are_up(self):
        return self.launch_process and self.launch_process.is_alive()


    # implement roslaunch boot.launch
    def launch_nodes(self):
        package = 'structure'
        executable = ''
        node = roslaunch.core.Node(package, executable)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        self.launch_process = launch.launch(node)


    def reset(self):
        rospy.loginfo(rospy.get_caller_id() + ": Resetting nodes")
        if self.nodes_are_up():
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


    def stop(self):
        rospy.loginfo(rospy.get_caller_id() + ": Stopping nodes")
        self.launch_process.stop()


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
