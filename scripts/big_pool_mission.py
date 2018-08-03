#!/usr/bin/env python
import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import time

from navigation_client import Command

# setup the command module
rospy.init_node('navigation_client')
co = Command()

target_heading1 = 15.
time_out1 = 55
target_depth1 = 1
target_heading2 = 359.
time_out2 = 115
target_depth2 = 2

try:
    # start the mission
    rospy.sleep(20.)
    co.begin()
    # mission specifications
    # first leg
    co.dh_change(target_depth1, target_heading1, 20)
    co.set_rc_velocity(1650, 1500, target_heading1, target_depth1, time_out1)

    # second leg
    co.dh_change(target_depth2, target_heading2, 20)
    co.set_rc_velocity(1650, 1500, target_heading2, target_depth2, time_out2)

except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted")
finally:
    # make sure communication is terminated cleanly
    co.finished()
