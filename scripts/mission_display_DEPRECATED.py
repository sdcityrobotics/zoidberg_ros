#!/usr/bin/env python

import roslib
import rospy
import time
import cv2

from zed_listener_DEPRECATED import Zed_Listener

zed = Zed_Listener()

try:
    zed.listener()

except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted")
