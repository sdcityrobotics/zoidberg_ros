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
from zoidberg_nav.msg import DVL
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
        self.zchannel = 2
        self.rchannel = 3
        self.xchannel = 4
        self.ychannel = 5
        # initilize current state to nonsense values
        self.curr_depth = -1.
        self.curr_heading = -1.


    def _set_task(self, goal):
        """Parse goal ID and send to correct handler"""
        if goal.actionID == 'depth_change':
            self.depth_change(goal)
        elif goal.actionID == 'heading_change':
            self.heading_change(goal)
        elif goal.actionID == 'set_rcvel':
            self.set_rcvel(goal)
        elif goal.actionID == 'arm':
            self.arm(goal.arm)
        elif goal.actionID == 'rc_off':
            self.rc_off()
        else:
            rospy.loginfo('%s actionID not recognized'%goal.actionID)


    def arm(self, is_armed):
        """Change the arm state of vehicle, set to Boolean value"""
        self.armer(value=is_armed)
        result = MoveRobotResult(actionID='arm',
                                 arm=is_armed)
        self._as.set_succeeded(result=result)


    def depth_change(self, goal):
        """Proportional control to change depth"""
        #self.mode_setter(base_mode=0, custom_mode='MANUAL')
        target_depth = goal.target_depth
        while not abs(target_depth - self.curr_depth) < self.depth_tol:
            # compute proportional controller output
            if self._as.is_preempt_requested():
                rospy.loginfo('Dive preempted')
                self._as.set_preempted()
                break
            # send command to RC channel
            channels = [1500] * 8

            zout = self._get_depth_pwm(goal.target_depth)
            channels[self.zchannel] = zout
            controlout = OverrideRCIn(channels=channels)
            self.contolp.publish(controlout)

            # send command feedback
            self.send_feedback(goal)
            self.rate.sleep()

        # publish a result message when finished
        if abs(ddiff) < self.depth_tol:
            self.rc_off()
            result = MoveRobotResult(actionID=goal.actionID,
                                     end_depth=goal.target_depth)
            self._as.set_succeeded(result=result)


    def _get_depth_pwm(self, target_depth):
        """Get PWM to get to desired depth"""
        ddiff = target_depth - self.curr_depth
        zout = ddiff * self.depth_p
        # limit output if necassary
        if abs(zout) > self.depth_pmax:
            if zout < 0:
                zout = -self.depth_pmax
            else:
                zout = self.depth_pmax
        zout += self.pwm_center
        return zout


    def heading_change(self, goal):
        """Proportional control to change heading"""
        #self.mode_setter(base_mode=0, custom_mode='MANUAL')
        hdiff = goal.target_heading - self.curr_heading
        while not abs(hdiff) < self.heading_tol:
            # compute proportional controller output
            pout = hdiff * self.heading_p
            # limit output if necassary
            if abs(pout) > self.heading_pmax:
                if pout < 0:
                    pout = -self.heading_pmax
                else:
                    pout = self.heading_pmax
            if self._as.is_preempt_requested():
                rospy.loginfo('Turn preempted')
                self._as.set_preempted()
                is_success = False
                break
            pout += self.pwm_center
            # send command to RC channel
            channels = [1500] * 8
            channels[self.rchannel] = pout
            controlout = OverrideRCIn(channels=channels)
            self.contolp.publish(controlout)

            # send command feedback
            self.send_feedback(goal)
            self.rate.sleep()
            hdiff = goal.target_heading - self.curr_heading


        if abs(hdiff) < self.heading_tol:
            self.rc_off()
            result = MoveRobotResult(actionID=goal.actionID,
                                        end_heading=goal.target_heading)
            self._as.set_succeeded(result=result)


    def set_rcvel(self, goal):
        """Set a constant velocity to motor"""
        target_depth = self.curr_depth
        xrc_cmd = goal.x_rc_vel
        yrc_cmd = goal.y_rc_vel
        #self.mode_setter(base_mode=0, custom_mode='ALT_HOLD')
        #self.mode_setter(base_mode=0, custom_mode='MANUAL')
        #self.mode_setter(base_mode=0, custom_mode='STABILIZE')
        # check that command velocity is resonable
        if xrc_cmd > self.pwm_center + self.xdiffmax \
                or xrc_cmd < self.pwm_center - self.xdiffmax:
            raise(ValueError('x goal velocity must be between %i and %i'%(
                    self.pwm_center + self.xdiffmax,
                    self.pwm_center - self.xdiffmax)))
        if yrc_cmd > self.pwm_center + self.ydiffmax \
                or yrc_cmd < self.pwm_center - self.ydiffmax:
            raise(ValueError('y goal velocity must be between %i and %i'%(
                    self.pwm_center + self.ydiffmax,
                    self.pwm_center - self.ydiffmax)))

        # send command to RC channel
        channels = [1500] * 8
        channels[self.xchannel] = xrc_cmd
        channels[self.ychannel] = yrc_cmd
        zout = self._get_depth_pwm(target_depth)
        channels[self.zchannel] = zout

        controlout = OverrideRCIn(channels=channels)

        while True:
            if self._as.is_preempt_requested():
                #self.mode_setter(base_mode=0, custom_mode='MANUAL')
                rospy.loginfo('RC set preempted')
                self._as.set_preempted()
                break

            self.contolp.publish(controlout)
            self.send_feedback(goal)
            self.rate.sleep()


    def rc_off(self):
        """Need to clear rc channel after a goal"""
        # reset control values
        channels = [1500] * 8
        controlout = OverrideRCIn(channels=channels)
        self.contolp.publish(controlout)
        self.rate.sleep()
        controlout = OverrideRCIn(channels=channels)
        self.contolp.publish(controlout)



    def send_feedback(self, goal):
        """Send feedback about current robot state"""
        fb = MoveRobotFeedback(actionID=goal.actionID,
                               current_depth=self.curr_depth,
                               current_heading=self.curr_heading)
        self._as.publish_feedback(fb)


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
