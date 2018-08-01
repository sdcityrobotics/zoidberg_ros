#!/usr/bin/env python
import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import actionlib
import time

from std_msgs.msg import String, Header
from mavros_msgs.msg import State
from zoidberg_nav.msg import MoveRobotAction, MoveRobotGoal

class Command():
    """Execute basic navigation commands"""
    def __init__(self):
        """Setup a number of servers used in navigation"""

        self._ac = actionlib.SimpleActionClient('helm/move_robot',
                                                 MoveRobotAction)
        self._ac.wait_for_server()

    def depth_change(self, target_depth, timeout):
        """command a depth change
            target_depth: depth in meters, positive is down from surface
        """
        goal = MoveRobotGoal(actionID='depth_change',
                               target_depth=target_depth)
        self._ac.send_goal(goal)
        to = rospy.Duration(secs=timeout)
        res = self._ac.wait_for_result(to)
        if not res:
            goal = MoveRobotGoal(actionID='rc_off')
            self._ac.send_goal(goal)
            rospy.loginfo("Depth change timed out")


    def heading_change(self, target_heading, timeout):
        """command a heading change
           target_heading: desired heading in Degrees, magnetic north
        """
        goal = MoveRobotGoal(actionID='heading_change',
                               target_heading=target_heading)
        self._ac.send_goal(goal)
        to = rospy.Duration(secs=timeout)
        res = self._ac.wait_for_result(to)
        if not res:
            goal = MoveRobotGoal(actionID='rc_off')
            self._ac.send_goal(goal)
            rospy.loginfo("Heading change timed out")

    def set_rc_velocity(self, x_rc_vel, y_rc_vel, timeout):
        """Set fixed velocities"""
        goal = MoveRobotGoal(actionID='set_rcvel',
                             x_rc_vel=x_rc_vel,
                             y_rc_vel=y_rc_vel)
        self._ac.send_goal(goal)
        to = rospy.Duration(secs=timeout)
        res = self._ac.wait_for_result(to)
        if not res:
            goal = MoveRobotGoal(actionID='rc_off')
            self._ac.send_goal(goal)
            rospy.loginfo("Velocity set timed out")

    def begin(self):
        """arm vehicle to begin moving"""
        goal = MoveRobotGoal(actionID='arm', arm=True)
        self._ac.send_goal(goal)
        to = rospy.Duration(secs=1.)
        res = self._ac.wait_for_result(to)

    def finished(self):
        """Shut down server"""
        goal = MoveRobotGoal(actionID='arm', arm=False)
        self._ac.send_goal(goal)
        to = rospy.Duration(secs=1.)
        res = self._ac.wait_for_result(to)
        self._ac.cancel_all_goals()


if __name__ == '__main__':
    try:
        rospy.init_node('navigation_client')
        co = Command()
        co.begin()
        #co.depth_change(.3, 1)
        #co.heading_change(280, 1)
        co.set_rc_velocity(1550, 1500, 1)
        #co.depth_change(1.5, 3)
        co.finished()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
