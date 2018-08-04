#!/usr/bin/env python
"""
=================
Navigation Server
=================
Provides basic behaviors used to build up a mission. Designed to be interfaced
by the Navigation client. The server is the helmsman, the client is the
captain. Breaking up the mission into a sucsession of basic behaviors makes it
simpler to ensure that each behavior works as expected.
The server-client terminology is taken from ROS actionlib package, which is
designed to remove the need for multi-threading programming typically required
in mission specifications. This is done by having the client provide a
feedback_callback, which is executed each time the server publishes a feedback
message.
"""

import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import actionlib
import time

from zoidberg_nav.msg import (MoveRobotAction, MoveRobotResult,
                              MoveRobotFeedback)
from zoidberg_nav.msg import DVL, VISION
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import StreamRate, CommandBool, SetMode

class NavigationServer:
    """Provide basic navigation capabilities"""
    def __init__(self):
        """Setup possible tasks"""
        # Start up action server ROS node
        self._as = actionlib.SimpleActionServer('~move_robot',
                                                   MoveRobotAction,
                                                   execute_cb=self._set_task,
                                                   auto_start=False)
        self._as.start()
        # set pixhawk stream rate to 10 Hz
        s1 = rospy.ServiceProxy('/apm/set_stream_rate', StreamRate)
        s1(stream_id=0, message_rate=10, on_off=True)
        #self.mode_setter = rospy.ServiceProxy('/apm/set_mode', SetMode)
        self.armer = rospy.ServiceProxy('/apm/cmd/arming', CommandBool)
        # channels where the server looks for necassary information
        rospy.Subscriber("/depth", FluidPressure, self._set_curr_depth)
        rospy.Subscriber("/heading", Float64, self._set_curr_heading)
	rospy.Subscriber("/objectCoordinates", VISION, self._set_object_coords)
        # channels where the server publishes control commands
        self.contolp = rospy.Publisher("/control",
                                       OverrideRCIn,
                                       queue_size=10)
        # publish comand rate, Hertz
        self.rate = rospy.Rate(rospy.get_param('~controller_frequency'))
        self.pwm_center = 1500
        # depth proportional controlled parameters
        gains = rospy.get_param('~depth')
        self.depth_p = gains['P']
        self.depth_pmax = gains['Pmax']
        self.depth_tol = gains['tolerance']
        # heading proportional controlled parameters
        gains = rospy.get_param('~heading')
        self.heading_p = gains['P']
        self.heading_pmax = gains['Pmax']
        self.heading_tol = gains['tolerance']
        gains = rospy.get_param('~vel')
        self.xdiffmax = gains['xdiffmax']
        self.ydiffmax = gains['ydiffmax']
        gains = rospy.get_param('~object')
        self.obj_p = gains['P']
        self.obj_pmax = gains['Pmax']
        self.framecenter = gains['framecenter']
        self.maxwidth = gains['maxwidth']
        self.objcycles = 50
        self.zchannel = 2
        self.rchannel = 3
        self.xchannel = 4
        self.ychannel = 5
        # initilize current state to nonsense values
        self.curr_depth = -1.
        self.curr_heading = -1.
	self.object_x = -1.
	self.object_y = -1.
        self.x_velocity = 9999
        self.y_velocity = 9999
        self.altitude = 9999


    def _set_task(self, goal):
        """Parse goal ID and send to correct handler"""
        if goal.actionID == 'dh_change':
            self.dh_change(goal)
        elif goal.actionID == 'set_rcvel':
            self.set_rcvel(goal)
        elif goal.actionID == 'gate_pass':
            self.gate_pass(goal)
        elif goal.actionID == 'arm':
            self.arm(goal.arm)
        elif goal.actionID == 'rc_off':
            self.rc_off()
        else:
            rospy.loginfo('%s actionID not recognized'%goal.actionID)


    def behavior_loop(self, goal, rc_cb, terminate_cb):
        """callbacks provide rc commands and termination condition"""
        rc_out = rc_cb(goal)
        is_term = terminate_cb(goal)

        while not is_term:
            # timeout termination behvior
            if self._as.is_preempt_requested():
                self.rc_off()
                rospy.loginfo('Timeout preempted')
                self._as.set_preempted()
                break

            controlout = OverrideRCIn(channels=rc_out)
            self.contolp.publish(controlout)

            # send command feedback
            self.send_feedback(goal)
            self.rate.sleep()

            # set up next loop
            is_term = terminate_cb(goal)
            rc_out = rc_cb(goal)

        # publish a result message when finished
        if depth_ok and heading_ok:
            self.rc_off()
            result = MoveRobotResult(actionID=goal.actionID,
                                     end_heading=self.curr_heading,
                                     end_depth=self.curr_depth)
            self._as.set_succeeded(result=result)


    def dh_change(self, goal):
        """ Change to specified depth and heading"""
        self.behavior_loop(goal, depth_heading_rc, depth_heading_isterm)

    def set_rcvel(self, goal):
        """Move with constant velocity holding depth and heading"""
        # Do not terminate this behavior, rather it is expected to timeout
        is_term = lambda g: True
        xrc_cmd = goal.x_rc_vel
        yrc_cmd = goal.y_rc_vel

        def rc_with_vels(self, goal):
            """Add constant rc commands to depth and heading hold"""
            channels = self.depth_heading_rc(goal)
            channels[self.xchannel] = xrc_cmd
            channels[self.ychannel] = yrc_cmd
            return channels

        self.behavior_loop(goal, rc_with_vels, is_term)

    def gate_pass(self, goal):
        """Center on gate untill it reaches a size threshold"""
        is_term = lambda g: True
        xrc_cmd = goal.x_rc_vel

        def rc_to_obj(self, goal):
            """Add constant rc commands to depth and heading hold"""
            channels = self.depth_heading_rc(goal)
            channels[self.xchannel] = xrc_cmd
            yrc_cmd = get_obj_pwm(self, goal)
            channels[self.ychannel] = yrc_cmd
            return channels

        def is_term(self, goal):
            """terminate when the gate is a certain size"""
            return self.object_width > self.maxwidth

        self.behavior_loop(goal, rc_to_obj, is_term)


    def depth_heading_rc(self, goal):
        """Proportional control to change depth"""
        target_depth = goal.target_depth
        target_heading = goal.target_heading
        # initialize empty RC command
        channels = [1500] * 8
        # compute heading and depth changes
        zout = self.get_depth_pwm(goal)
        hout = self.get_heading_pwm(goal)

        channels[self.zchannel] = zout
        channels[self.rchannel] = hout
        return channels


    def depth_heading_isterm(self, goal):
        """Termination condition for heading moveto"""
        depth_ok = abs(target_depth - self.curr_depth) < self.depth_tol
        heading_ok = abs(target_heading - self.curr_heading)\
                        < self.heading_tol
        return depth_ok and heading_ok


    def arm(self, is_armed):
        """Change the arm state of vehicle, set to Boolean value"""
        self.armer(value=is_armed)
        result = MoveRobotResult(actionID='arm',
                                 arm=is_armed)
        self._as.set_succeeded(result=result)


    def rc_off(self):
        """Need to clear rc channel after a goal"""
        # reset control values
        channels = [1500] * 8
        controlout = OverrideRCIn(channels=channels)
        self.contolp.publish(controlout)
        self.rate.sleep()
        # send twice to make sure
        controlout = OverrideRCIn(channels=channels)
        self.contolp.publish(controlout)


    def send_feedback(self, goal):
        """Send feedback about current robot state"""
        fb = MoveRobotFeedback(actionID=goal.actionID,
                               current_depth=self.curr_depth,
                               current_heading=self.curr_heading)
        self._as.publish_feedback(fb)


    def get_depth_pwm(self, goal):
        """Get PWM to get to desired depth"""
        ddiff = goal.target_depth - self.curr_depth
        zout = ddiff * self.depth_p
        # limit output if necassary
        if abs(zout) > self.depth_pmax:
            if zout < 0:
                zout = -self.depth_pmax
            else:
                zout = self.depth_pmax
        zout += self.pwm_center
        return zout


    def get_obj_pwm(self, goal):
        """Get PWM to get to desired depth"""
        odiff = self.framecenter - self.object_x
        yout = odiff * self.obj_p
        # limit output if necassary
        if abs(yout) > self.obj_pmax:
            if yout < 0:
                yout = -self.obj_pmax
            else:
                yout = self.obj_pmax
        yout += self.pwm_center
        return yout

    def get_heading_pwm(self, goal):
        """Get PWM to get to desired heading"""
        hdiff = goal.target_heading - self.curr_heading
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
            if hout < 0:
                hout = -self.heading_pmax
            else:
                hout = self.heading_pmax
        hout += self.pwm_center
        return hout


    def _set_object_coords(self, object_coords):
        """Set object x and y coordinates"""
	self.object_x = object_coords.x_coord
	self.object_y = object_coords.y_coord
	self.object_width = object_coords.width


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
    rospy.init_node('navigation_server')
    server = NavigationServer()
    rospy.spin()
