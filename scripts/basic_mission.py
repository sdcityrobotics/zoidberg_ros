#!/usr/bin/env python
import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import time

from navigation_client import Command

# setup the command module
rospy.init_node('navigation_client')
co = Command()

try:
    # start the mission
    co.begin()
    # mission specifications
    co.depth_change(.2, 50)
    co.heading_change(270, 3)
    co.set_rc_velocity(1550, 1500, 50)
except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted")
finally:
    # make sure communication is terminated cleanly
    co.finished()
