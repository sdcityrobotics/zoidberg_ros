#!/usr/bin/env python
import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import time

from navigation_client import Command

# setup the command module
rospy.init_node('navigation_client')
co = Command()

#time_gp = 25
#time_out1 = 30
#time_gp_extra = 5

target_heading1 = 151.
time_out1 = 60
target_depth1 = 1

# 45 degrees off initial heading
target_heading2 = 195.5
time_out2 = 110
target_depth2 = 2.07

target_heading3 = 141.
time_out3 = 30
target_depth3 = 2

try:
    # start the mission
    rospy.sleep(20.)
    co.begin()
    # mission specifications
    # first leg
    co.dh_change(target_depth1, target_heading1, 20)
    # 1650 is a safe speed
    co.set_rc_velocity(1650, 1500,
                       target_depth1, target_heading1,
                       time_out1)
    #is_togate = co.gate_pass(1650, 1500,
                             #target_depth1, target_heading1,
                             #time_gp)
    # if gate_pass exits cleanly, drive for a bit to pass through gate
    #if is_togate:
        #co.set_rc_velocity(1650, 1500,
                           #target_depth1,
                           #target_heading1,
                           #time_gp_extra)

    # second leg
    co.dh_change(target_depth2, target_heading2, 20)
    co.set_rc_velocity(1650, 1500, target_depth2, target_heading2, time_out2)

    co.dh_change(target_depth3, target_heading3, 20)
    co.set_rc_velocity(1650, 1500, target_depth3, target_heading3, time_out3)

except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted")
finally:
    # make sure communication is terminated cleanly
    co.finished()
