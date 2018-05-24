#!/usr/bin/env python
# basic ropy tutorial
import rospy
from std_msgs.msg import String
from zoidberg_nav import DepthChange

def commander():
    pub = rospy.Publisher('zoidberg/String', String, queue_size=10)
    rospy.init_node('commander', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        hello_str = "time is %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass
