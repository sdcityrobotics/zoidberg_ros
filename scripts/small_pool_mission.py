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
    #rospy.sleep(2.)
    co.begin()
    # mission specifications
    co.dh_change(.2, 55.7, 5)
    #co.set_rc_velocity(1525, 1500, .2, 270, 10)
    #co.gate_pass(1500, .2, 10, 50)
    #co.object_center(1500, 10, 50)
except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted")
finally:
    # make sure communication is terminated cleanly
    co.finished()
