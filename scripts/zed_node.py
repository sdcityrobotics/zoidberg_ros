#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ZedListener:
    def __init__(self):
        self.bridge = CvBridge()
	self.image = None

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image = cv_image

    def listen(self):
        rospy.Subscriber("zed/rgb/image_raw_color", Image, self.callback)

    def getImage(self):
        return self.image
