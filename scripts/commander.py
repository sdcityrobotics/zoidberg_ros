#!/usr/bin/env python
# basic ropy tutorial
import rospy
from std_msgs.msg import String, Header
from zoidberg_nav.msg import DepthChange

def commander():
    pubdc = rospy.Publisher('zoidberg/DepthChange', DepthChange, queue_size=10)
    rospy.init_node('commander', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    publish_time = 0.
    publish_duration = 5.
    targetdepth = 0.5
    while not rospy.is_shutdown():
        curr_time = rospy.get_time()
        if publish_time + publish_duration < curr_time:
		h = Header(stamp=rospy.Time.now())
                publish_time = curr_time
                Timeout = rospy.Duration(secs=publish_duration)
                dc = DepthChange(header=h, start_time=publish_time, timeout=Timeout,
                                 target_depth=targetdepth)
                rospy.loginfo(dc)
                pubdc.publish(dc)
        rate.sleep()

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass
