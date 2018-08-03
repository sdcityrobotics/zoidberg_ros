#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Header
import cv2
from zoidberg_nav.msg import vision

class ZedListener:
    def __init__(self):
        self.bridge = CvBridge()
        
    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image = cv_image

    def listen(self):
        rospy.Subscriber("zed/rgb/image_raw_color", Image, self.callback)
        rospy.spinOnce()

    def getImage(self):
        self.listener()
        return self.image

class ZedTalker:
    def __init__(self):
        self.pub = rospy.Publisher('objectCoordinates', String, queue_size=10)

    def talk(data):
        rospy.loginfo(data)
        msg = vision()
        msg.x_coord = data.x
        msg.y_coord = data.y
        pub.publish(msg)