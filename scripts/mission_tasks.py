#!/usr/bin/env python
import rospy
import time
import cv2
import zed_node


class MissionTasks:
    def __init__(self):
        self.zedListener = ZedListener()
        self.zedListener.listen()
        self.zedTalker = ZedTalker()
    
    def missionControl(self):
        # THIS CAN BE MOVED TO MAIN CONTROL IF NEEDED
        #find gate
        self.doTask(30, "gate") #change time as needed

    def doTask(self, seconds, task):
        self.task = task
        count = 0
        while count < seconds:
            try:
                image = self.zedListener.getImage()
                coords = self.processImage(task, image)
                self.zedTalker.talk(coords)
                time.sleep(1)
            except rospy.ROSInterruptException:
                rospy.loginfo("Program interrupted")
    
    def processImage(self, task, image):
        if task == "gate":
            # TO-DO: find gate code
        else if task == "dice":
            # TO-DO: find dice code

if __name__ == '__main__':
    mission = MissionTasks()
    try:
        mission.missionControl()
    except rospy.ROSInterruptException:
        pass
