#!/usr/bin/env python
import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import time

from navigation_client import Command


try:
    rospy.init_node('navigation_client')
    co = Command()
    co.begin()
    #co.depth_change(.3, 3)
    #co.heading_change(280, 1)
    #co.set_rc_velocity(1550, 1500, 3)
    co.set_rc_velocity(1500, 1500, 3)
    #co.depth_change(1.5, 3)
    co.finished()
except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted")
