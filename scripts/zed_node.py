#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, ChannelFloat32
from cv_bridge import CvBridge

class ZedListener:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.depth = None

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image = cv_image

    def callbackDepth(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        self.depth = cv_image
    
    def listen(self):
        rospy.Subscriber("zed/rgb/image_raw_color", Image, self.callback)
        rospy.Subscriber("zed/depth/depth_registered", Image, self.callbackDepth)

    def getImage(self):
        return self.image

    def getDepth(self):
        return self.depth
