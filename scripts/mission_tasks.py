#!/usr/bin/env python
import rospy
import time
import cv2
import zed_node
import vision

class MissionTasks:
    def __init__(self):
        self.zedListener = ZedListener()
        self.zedListener.listen()
        self.zedTalker = ZedTalker()
        self.vision = Vision()
    
    def missionControl(self):
        # THIS CAN BE MOVED TO MAIN CONTROL IF NEEDED
        #find gate
        self.doTask(30, "gate") #change time as needed
        #find dice
        #self.doTask(30, "dice") #change time as needed

    def doTask(self, seconds, task):
        self.task = task
        count = 0
        while count < seconds:
            try:
                image = self.zedListener.getImage()
                coords = self.processImage(task, image)
                self.zedTalker.talk(coords)
                count += 1
                time.sleep(1)
            except rospy.ROSInterruptException:
                rospy.loginfo("Program interrupted")
    
    def processImage(self, task, image):
        if task == "gate":
            (x,y) = self.vision.findGate(image)
        else if task == "dice":
            (x,y) = self.vision.findDice(image)
        return (x,y)

if __name__ == '__main__':
    rospy.init_node('mission_tasks')
    mission = MissionTasks()
    try:
        mission.missionControl()
    except rospy.ROSInterruptException:
        pass
