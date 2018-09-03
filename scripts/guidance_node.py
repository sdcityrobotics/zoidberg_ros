#!/usr/bin/env python
"""
Guidance node
=============
User specifies desired depth, heading and velocity of robot. Guidance works
with control system on Pixhawk to make this happen.
"""

import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import actionlib
import time
from math import copysign

from zoidberg_nav.msg import (MoveRobotAction, MoveRobotResult,
                              MoveRobotFeedback)
from zoidberg_nav.msg import DVL, VISION
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import StreamRate, CommandBool, SetMode

class Guidance:
    """Hold slow changing target values of depth, heading and velocity"""
    def __init__(self):
        """Initialize an empty guidance node"""
        self.target_depth = None
        self.target_heading = None
        self.target_vx = None
        self.target_vy = None

        rospy.Subscriber("/depth", FluidPressure, self._set_curr_depth)
        rospy.Subscriber("/heading", Float64, self._set_curr_heading)
        # channels where guidance publishes control commands
        self.contolp = rospy.Publisher("/control",
                                       OverrideRCIn,
                                       queue_size=10)
        # publish comand rate, Hertz
        self.rate = rospy.Rate(rospy.get_param('~controller_frequency'))
        self.pwm_center = 1500

        # depth guidance parameters
        gains = rospy.get_param('~depth')
        self.depth_p = gains['P']
        self.depth_pmax = gains['Pmax']
        # heading guidance parameters
        gains = rospy.get_param('~heading')
        self.heading_p = gains['P']
        self.heading_pmax = gains['Pmax']
        # x-velocity guidance parameters
        gains = rospy.get_param('~xvel')
        self.vx_p = gains['P']
        self.vx_pmax = gains['Pmax']
        # y-velocity guidance parameters
        gains = rospy.get_param('~yvel')
        self.vy_p = gains['P']
        self.vy_pmax = gains['Pmax']

        self.zchannel = 2
        self.rchannel = 3
        self.xchannel = 4
        self.ychannel = 5
        # initilize current state to nonsense values
        self.curr_depth = -9999
        self.curr_heading = -9999.
	self.curr_vx = -9999.
	self.curr_vy = -9999.

    def __call__(self):
        """Compute controll unit inputs"""
        pass

    def get_heading_pwm(self):
        """Get PWM to get to desired heading"""
        hdiff = self.target_heading - self.curr_heading
        # handle 0/360 change at magnetic north
        if abs(hdiff) > 180:
            if hdiff < 0:
                hdiff += 360
            else:
                hdiff -= 360
        # p-control
        hout = hdiff * self.heading_p
        # limit output if necassary
        if abs(hout) > self.heading_pmax:
            hout = copysign(self.heading_pmax, hout)
        hout += self.pwm_center
        return hout

    def get_depth_pwm(self):
        """Get PWM to get to desired depth"""
        ddiff = self.target_depth - self.curr_depth
        if self.curr_depth < -1:
            rospy.loginfo('Depth sensor is not initialized')
            return self.pwm_center
        zout = ddiff * self.depth_p
        # limit output if necassary
        if abs(zout) > self.depth_pmax:
            zout = copysign(self.depth_pmax, zout)
        zout += self.pwm_center
        return zout

    def _set_curr_depth(self, curr_depth):
        """Set the current depth when it is published"""
        self.curr_depth = curr_depth.fluid_pressure


    def _set_curr_heading(self, curr_heading):
        """Set the current depth when it is published"""
        self.curr_heading = curr_heading.data


    def _set_curr_pose(self, dvl_output):
        """Set the currunt position, velocity and altitude from DVL"""
        pass

if __name__ == '__main__':
    rospy.init_node('guidance_node')
    server = Guidance()
    rospy.spin()
