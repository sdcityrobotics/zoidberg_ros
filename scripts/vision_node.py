#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import time
import cv2
from zed_node import ZedListener
from vision_tasks import VisionTasks
from zoidberg_ros.msg import Vision
import numpy as np

class MissionTasks:
    def __init__(self):
        self.zedListener = ZedListener()
	self.zedListener.listen()
        self.vision = VisionTasks()
        self.pub = rospy.Publisher('objectCoordinates', Vision, queue_size=10)
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
                depth = self.zedListener.getDepth()
                if image is not None:
                    coords = self.processImage(task, image)
                    img, x, y, w = coords
                    meanDepth = -1
                    if depth is not None:
                        meanDepth = depth[y, x]
                    self.talk(coords, meanDepth)
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("Program interrupted")

    def processImage(self, task, image):
        if task == "gate":
            coords = self.vision.findGate(image)
        elif task == "dice":
            coords = self.vision.findDice(image)
        return coords

    def talk(self, coords, depth):
        msg = Vision()
        img, x, y, w = coords
        cv2.imshow("image", img)
        cv2.waitKey(1)
        msg.px_coord = x
        msg.py_coord = y
        msg.delta_px = w
        msg.depth = depth
        rospy.loginfo(msg)
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('mission_tasks')
    mission = MissionTasks()
    try:
        mission.missionControl()
    except rospy.ROSInterruptException:
        pass
