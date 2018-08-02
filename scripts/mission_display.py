#!/usr/bin/env python

import roslib
import rospy
import time
import cv2

from zed_listener import Zed_Listener

zed = Zed_Listener()

try:
    image = zed.getImage()
    while(1):
    #zed.listener()
        cv2.imshow('image', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted")
