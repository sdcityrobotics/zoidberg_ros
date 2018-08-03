#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ZedListener:
    def __init__(self):
<<<<<<< Updated upstream
        self.bridge = CvBridge()
=======
        pass
>>>>>>> Stashed changes

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image = cv_image

    def listen(self):
        rospy.Subscriber("zed/rgb/image_raw_color", Image, self.callback)
        rospy.spinOnce()

<<<<<<< Updated upstream
    def getImage(self):
        self.listener()
        return self.image
=======
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("zed/rgb/image_raw_color", Image, callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def talker(self, data):
        pub = rospy.Publisher('objectCoordinates', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rospy.loginfo(data)
            pub.publish(data)
            rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
>>>>>>> Stashed changes
