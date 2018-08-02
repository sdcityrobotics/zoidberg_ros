#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Header
from tasks_node import Tasks
import cv2

class Zed:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('objectCoordinates', String, queue_size=10)
        rospy.init_node('image_objects')
        
    def callback(self, idata, objective):
        cv_image = self.bridge.imgmsg_to_cv2(idata, "bgr8")
        cv2.imshow("image", cv_image)
        #tasks = Tasks(cv_image)
        #data = tasks.findObject(objective)
        
        #self.talker(data)

    def listener(self, task):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('zed_listener')
        cb = lambda idata: self.callback(self, idata, "qualGate")

        rospy.Subscriber("zed/rgb/image_raw_color", Image, cb)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    
    def talker(self, data):
        rospy.loginfo(data)
        self.pub.publish(data)

if __name__ == '__main__':
    zednnode = Zed()
    try:
        zednode.listener()
    except rospy.ROSInterruptException:
        pass