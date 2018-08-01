#!/usr/bin/env python
"""
======================
Read pressure data
======================
read scaledpressure2 from the pyhawk
"""

import rospy
from pymavlink import mavutil
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header

device = "udpin:127.0.0.1:14550"

def read_pressure(mav_obj):
    """
    Read accelerometer readings until taxis is exhausted.
    There will only be output once the total time has elapsed.
    """
    pub = rospy.Publisher('/depth', FluidPressure, queue_size=10)
    rospy.init_node('externalpressure')
    rate = rospy.Rate(10)
    msg_type = 'SCALED_PRESSURE'

    msg = mav_obj.recv_match(blocking=True)
    # flush out old data
    if msg.get_type() == "BAD_DATA":
        if mavutil.all_printable(msg.data):
            sys.stdout.write(msg.data)
            sys.stdout.flush()

    while not rospy.is_shutdown():
        msg = mav_obj.recv_match(type=msg_type, blocking=True)
        h = Header()
        h.stamp = rospy.Time.now()
        depth_m = (msg.press_abs - 1014.25) / 100
        fp = FluidPressure(header=h,
                           fluid_pressure=depth_m,
                           variance=0)
        pub.publish(fp)
        rate.sleep()

mav = mavutil.mavlink_connection(device,
                                 source_system=1,
                                 source_component=1)
# check that there is a heartbeat
mav.recv_match(type='HEARTBEAT', blocking=True)
print("Heartbeat from APM (system %u component %u)" %
      (mav.target_system, mav.target_component))
print('')


# a try block ensures that mav with always be closed
try:
    press = read_pressure(mav)
finally:
    mav.close()
