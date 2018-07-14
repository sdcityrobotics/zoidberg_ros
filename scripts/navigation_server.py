#!/usr/bin/env python
import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import actionlib
import time

from zoidberg_nav.msg import MoveRobotAction, MoveRobotResult, MoveRobotFeedback
from std_msgs.msg import Float64, Header
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import StreamRate, CommandBool

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
        self.armer = rospy.ServiceProxy('/apm/cmd/arming', CommandBool)
        # channels where the server looks for necassary information
        rospy.Subscriber("/depth", Float64, self._set_curr_depth)
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
        self.zchannel = 3
        self.rchannel = 4
        self.xchannel = 5
        self.ychannel = 6
        # initilize current state to nonsense values
        self.curr_depth = -1.
        self.curr_heading = -1.


    def _set_task(self, goal):
        """Parse goal ID and send to correct handler"""
        if goal.actionID == 'change_depth':
            self.change_depth(goal)
        elif goal.actionID == 'change_heading':
            self.change_heading(goal)
        elif goal.actionID == 'set_velocity':
            self.set_velocity(goal.xvel, goal.yvel)
        elif goal.actionID == 'arm':
            self.arm(goal.arm)
        else:
            rospy.loginfo('%s actionID not recognized'%goal.actionID)


    def arm(self, is_armed):
        """Change the arm state of vehicle, set to Boolean value"""
        self.armer(value=is_armed)
        result = MoveRobotResult(actionID='arm',
                                 arm=arm)
        self._as.set_succeeded(result=result)

    def set_velocity(self, xvel, yvel):
        """Simply set the velocity"""
        channels = [1500] * 8
        channels[self.xchannel] = xvel
        channels[self.ychannel] = yvel
        controlout = OverrideRCIn(channels=channels)
        self.contolp.publish(controlout)

    def change_depth(self, goal):
        """Proportional control to change depth"""
        target_depth = goal.target_depth

        ddiff = target_depth - self.curr_depth

        while not abs(ddiff) < self.depth_tol:
            # compute proportional controller output
            pout = ddiff * self.depth_p
            # limit output if necassary
            if abs(pout) > self.depth_pmax:
                if pout < 0:
                    pout = -self.depth_pmax
                else:
                    pout = self.depth_pmax
            if self._as.is_preempt_requested():
                rospy.loginfo('Dive preempted')
                self._as.set_preempted()
                break
            pout += self.pwm_center
            # send command to RC channel
            channels = [1500] * 8
            channels[self.zchannel] = pout
            controlout = OverrideRCIn(channels=channels)
            self.contolp.publish(controlout)

            # send command feedback
            self.send_feedback(goal)
            self.rate.sleep()
            ddiff = target_depth - self.curr_depth

        # reset control values
        channels = [1500] * 8
        controlout = OverrideRCIn(channels=channels)
        self.contolp.publish(controlout)

        # publish a result message when finished
        if abs(ddiff) < self.depth_tol:
            result = MoveRobotResult(actionID=goal.actionID,
                                     end_depth=goal.target_depth)
            self._as.set_succeeded(result=result)


    def change_heading(self, goal):
        """Proportional control to change heading"""
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

        # reset control values
        channels = [1500] * 8
        controlout = OverrideRCIn(channels=channels)
        self.contolp.publish(controlout)

        if abs(hdiff) < self.heading_tol:
            result = MoveRobotResult(actionID=goal.actionID,
                                        end_heading=goal.target_heading)
            self._as.set_succeeded(result=result)


    def send_feedback(self, goal):
        """Send feedback about current robot state"""
        fb = MoveRobotFeedback(actionID=goal.actionID,
                               current_depth=self.curr_depth,
                               current_heading=self.curr_heading)
        self._as.publish_feedback(fb)


    def _set_curr_depth(self, curr_depth):
        """Set the current depth when it is published"""
        self.curr_depth = curr_depth.data


    def _set_curr_heading(self, curr_heading):
        """Set the current depth when it is published"""
        self.curr_heading = curr_heading.data

    def _set_curr_pose(self, dvl_output):
        """Set the currunt position, velocity and altitude from DVL"""
        pass

if __name__ == '__main__':
    rospy.init_node('navigation_server')
    server = NavigationServer()
    server.arm(True)
    rospy.spin()
