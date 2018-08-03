#!/usr/bin/env python
import rospy
import time
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
        self.doTask(100, "gate") #change time as needed
        #find dice
        #self.doTask(300, "dice") #change time as needed

    def doTask(self, iterations, task):
        self.rate = rospy.Rate(10)  # 10 Hz
        count = 0
        while count < iterations:
            try:
                image = self.zedListener.getImage()
		if image is not None:
                	coords = self.processImage(task, image)
                	self.talk(coords)
                count += 1
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
	x, y = data
        msg.x_coord = x
        msg.y_coord = y
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('mission_tasks')
    mission = MissionTasks()
    try:
        mission.missionControl()
    except rospy.ROSInterruptException:
        pass
