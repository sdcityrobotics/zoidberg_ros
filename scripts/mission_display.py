#!/usr/bin/env python

import roslib
import rospy
import time
import cv2

from zed_listener import Zed_Listener

rospy.init_node('display_mission')
zed = Zed_Listener()

try:
    while(1):
        zed.listener()
        image = zed.getImage()
        cv2.imshow('image', image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
except rospy.ROSInterruptException:
    rospy.loginfo("Program interrupted")
