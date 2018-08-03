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


    def dh_change(self, target_depth, target_heading, timeout):
        """
        command a heading change
        target_heading: desired heading in Degrees, magnetic north
        target_depth: desired depth in meters
        """
        goal = MoveRobotGoal(actionID='dh_change',
                               target_heading=target_heading,
                               target_depth=target_depth)
        self._ac.send_goal(goal)
        to = rospy.Duration(secs=timeout)
        res = self._ac.wait_for_result(to)
        if not res:
            goal = MoveRobotGoal(actionID='rc_off')
            self._ac.send_goal(goal)
            rospy.loginfo("Heading change timed out")

    def set_rc_velocity(self, x_rc_vel, y_rc_vel,target_heading,
            target_depth, timeout):
        """Set fixed velocities"""
        goal = MoveRobotGoal(actionID='set_rcvel',
                             x_rc_vel=x_rc_vel,
                             y_rc_vel=y_rc_vel,
                             target_heading=target_heading,
                             target_depth=target_depth)
        self._ac.send_goal(goal)
        to = rospy.Duration(secs=timeout)
        res = self._ac.wait_for_result(to)
        if not res:
            goal = MoveRobotGoal(actionID='rc_off')
            self._ac.send_goal(goal)
            rospy.loginfo("Velocity set timed out")


    def gate_pass(self, x_rc_vel,target_heading, target_depth, timeout):
        """Set fixed velocities"""
        goal = MoveRobotGoal(actionID='gate_pass',
                             x_rc_vel=x_rc_vel,
                             target_heading=target_heading,
                             target_depth=target_depth)
        self._ac.send_goal(goal)
        to = rospy.Duration(secs=timeout)
        res = self._ac.wait_for_result(to)
        if not res:
            goal = MoveRobotGoal(actionID='rc_off')
            self._ac.send_goal(goal)
            rospy.loginfo("Gate pass set timed out")


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
        co.dh_change(0.3, 30, 20)
        #co.set_rc_velocity(1550, 1500, 1)
        #co.depth_change(1.5, 3)
        co.finished()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
