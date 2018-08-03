#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Header
import cv2

class Zed_Listener:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('zed_listener')
        
    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #self.image = cv_image
        cv2.imshow('frame', cv_image)
        cv2.waitKey(1)

    def listener(self):
        rospy.Subscriber("zed/rgb/image_raw_color", Image, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def getImage(self):
        self.listener()
        return self.image

if __name__ == '__main__':
    zednode = Zed_Listener()
    try:
        zednode.listener()
    except rospy.ROSInterruptException:
        pass