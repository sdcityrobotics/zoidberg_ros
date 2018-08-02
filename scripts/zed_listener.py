#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Header
import cv2

class Zed_Listener:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('objectCoordinates', String, queue_size=10)
        rospy.init_node('image_objects')
        
    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("image", cv_image)

    def listener(self):
        rospy.Subscriber("zed/rgb/image_raw_color", Image, data)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    zednode = Zed_Listener()
    try:
        zednode.listener("")
    except rospy.ROSInterruptException:
        pass