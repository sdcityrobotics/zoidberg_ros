#!/usr/bin/env python
import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import actionlib
import time

from zoidberg_nav.msg import MoveRobotAction, MoveRobotResult

class NavigationServer:
    """Provide basic navigation capabilities"""
    def __init__(self):
        """Setup possible tasks"""
        self.server = actionlib.SimpleActionServer('zoidberg_nav/move_robot',
                                                   MoveRobotAction,
                                                   execute_cb=self._set_task,
                                                   auto_start=False)
        self.server.start()


    def _set_task(self, goal):
        """Parse goal ID and send to correct handler"""
        if goal.actionID == 'change_depth':
            self.change_depth(goal)
        elif goal.actionID == 'change_heading':
            self.change_heading(goal)
        else:
            rospy.loginfo('%s actionID not recognized'%goal.actionID)
 

    def change_depth(self, goal):
        """Proportional control to change depth"""
        target_depth = goal.target_depth
        time.sleep(2.)
        result = MoveRobotResult(actionID=goal.actionID,
                                   end_depth=target_depth)
        self.server.set_succeeded(result=result)
  

    def change_heading(self, goal):
        """Proportional control to change heading"""
        target_heading = goal.target_heading
        time.sleep(2.)
        result = MoveRobotResult(actionID=goal.actionID,
                                   end_heading=target_heading)
        self.server.set_succeeded(result=result)

if __name__ == '__main__':
    rospy.init_node('navigation_server')
    server = NavigationServer()
    rospy.spin()
