#!/usr/bin/env python
import rospy
import time
import cv2
import numpy as np
from zed_node import ZedListener
from vision import VisionTasks
from zoidberg_nav.msg import VISION


class MissionTasks:
    def __init__(self):
        self.zedListener = ZedListener()
	self.zedListener.listen()
        self.vision = VisionTasks()
        self.pub = rospy.Publisher('objectCoordinates', VISION, queue_size=10)
        self.rate = None
    
    def missionControl(self):
        # THIS CAN BE MOVED TO MAIN CONTROL IF NEEDED
        #find gate
        self.doTask("gate") #change time as needed
        #find dice
        #self.doTask(300, "dice") #change time as needed

    def doTask(self, task):
        self.rate = rospy.Rate(10)  # 10 Hz
        while True:
            try:
                image = self.zedListener.getImage()
		if image is not None:
                	coords = self.processImage(task, image)
                	self.talk(coords)
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("Program interrupted")
    
    def processImage(self, task, image):
        if task == "gate":
            coords = self.vision.findGate(image)
        elif task == "dice":
            coords = self.vision.findDice(image)
        return coords

    def talk(self, data):
        #rospy.loginfo(x,y)
        msg = VISION()
        img, x, y, w = data
        cv2.imshow("image", img)
        cv2.waitKey(1)
        msg.x_coord = x
        msg.y_coord = y
        msg.width = w
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('mission_tasks')
    mission = MissionTasks()
    try:
        mission.missionControl()
    except rospy.ROSInterruptException:
        pass
