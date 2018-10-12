#!/usr/bin/env python
"""
Guidance node
=============
User specifies desired depth, heading and velocity of robot. Guidance works
with control system on Pixhawk to make this happen.
"""

from __future__ import print_function, division
import roslib
roslib.load_manifest('zoidberg_ros')
import rospy
from math import copysign
import zoidberg_ros.msg
import std_msgs.msg
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import StreamRate, CommandBool, SetMode

class Guidance:
    """Hold slow changing target values of depth, heading and velocity"""
    def __init__(self):
        """Initialize an empty guidance node"""
        self.desired_state = None
        self.current_state = None
        # channel where guidance listens for relevant information
        rospy.Subscriber("/navigation",
                         zoidberg_ros.msg.VehicleState,
                         self._set_curr_state)
        rospy.Subscriber("/guidance",
                         zoidberg_ros.msg.VehicleState,
                         self._set_curr_guidance)
        # guidance publishes control commands
        self.contolp = rospy.Publisher("/control",
                                       OverrideRCIn,
                                       queue_size=10)
        # publish comand rate, Hertz
        self.rate = rospy.Rate(10)
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
        # pixhawk motor channels
        self.zchannel = 2
        self.rchannel = 3
        self.xchannel = 4
        self.ychannel = 5

    def spin(self):
        """Compute control unit inputs at a constant frequency"""
        while not rospy.is_shutdown():
            # create a message to pass to the pixhawk
            channels = [1500] * 8
            # compute heading and depth changes
            channels[self.zchannel] = self.get_depth_pwm()
            channels[self.rchannel] = self.get_heading_pwm()
            channels[self.xchannel] = self.get_xvelocity_pwm()
            channels[self.ychannel] = self.get_yvelocity_pwm()
            # send message to pixhawk
            cmd = OverrideRCIn(channels=channels)
            # log control message and publish it
            rospy.loginfo(cmd)
            self.contolp.publish(cmd)
            # rest
            self.rate.sleep()

    def get_heading_pwm(self):
        """Get PWM to get to desired heading"""
        if self.current_state is None:
            rospy.loginfo('compass is not initialized')
            return self.pwm_center
        hdiff = self.desired_state.target_heading - self.current_state.curr_heading
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
        # add RC center of 1500
        hout += self.pwm_center
        return hout

    def get_depth_pwm(self):
        """Get PWM to get to desired depth"""
        if self.current_state is None or self.current_state.curr_depth < -1:
            rospy.loginfo('Depth sensor is not initialized')
            return self.pwm_center
        ddiff = self.desired_state.target_depth - self.current_state.curr_depth
        zout = ddiff * self.depth_p
        # limit output if necassary
        if abs(zout) > self.depth_pmax:
            zout = copysign(self.depth_pmax, zout)
        zout += self.pwm_center
        # add RC center of 1500
        return zout

    def get_xvelocity_pwm(self):
        """Get pwm that moves forward at roughly the desired speed
        TODO: Calibrate the velocity gain
        This behavior does not currently impliment feedback (from DVL) and
        is an approximation at best
        """
        if self.desired_state is None:
            rospy.loginfo('Desired state is not initialized')
            return self.pwm_center
        # This has no measurement imput !
        vxout = self.vx_p * self.desired_state.vx
        # limit output if necassary
        if abs(vxout) > self.vx_pmax:
            vxout = copysign(self.vx_pmax, vxout)
        # add RC center of 1500
        vxout += self.pwm_center
        return vxout

    def get_yvelocity_pwm(self):
        """Get pwm that moves sideways at roughly the desired speed
        TODO: Calibrate the velocity gain
        This behavior does not currently impliment feedback (from DVL) and
        is an approximation at best
        """
        if self.desired_state is None:
            rospy.loginfo('Desired state is not initialized')
            return self.pwm_center
        # This has no measurement imput !
        vyout = self.vy_p * self.desired_state.vy
        # limit output if necassary
        if abs(vyout) > self.vy_pmax:
            vyout = copysign(self.vy_pmax, vyout)
        # add RC center of 1500
        vyout += self.pwm_center
        return vyout

    def _set_curr_state(self, curr_state):
        """Set the current navigation when it is published"""
        self.current_state = curr_state

    def _set_curr_guidance(self, desired_state):
        """Set the current guidance when it is published"""
        self.desired_state = desired_state

if __name__ == '__main__':
    rospy.init_node('guidance_node')
    server = Guidance()
    # this try block taken from rospy tutorials for talker.py
    try:
        server.spin()
    except rospy.ROSInterruptException:
        pass
