#!/usr/bin/env python

import roslib
import rospy
import time
import cv2

from zed_listener import Zed_Listener

rospy.init_node('display_mission')
zed = Zed_Listener()

try:
    zed.listener(gate_callback)
except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted")

def gate_callback(data):
    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow('image', image)
    cv2.waitKey(1)