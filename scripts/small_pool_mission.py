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
    rospy.sleep(20.)
    co.begin()
    # mission specifications
    co.heading_change(300, 20)
    co.depth_change(.2, 20)
    co.heading_change(300, 20)
    co.set_rc_velocity(1550, 1500, 50)
except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted")
finally:
    # make sure communication is terminated cleanly
    co.finished()
