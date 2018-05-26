#!/usr/bin/env python
import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import actionlib
import time


from zoidberg_nav.msg import MoveRobotAction, MoveRobotResult, MoveRobotFeedback
from std_msgs.msg import Float64, Header
from mavros_msgs.msg import ManualControl

class NavigationServer:
    """Provide basic navigation capabilities"""
    def __init__(self):
        """Setup possible tasks"""
        self._as = actionlib.SimpleActionServer('zoidberg_nav/move_robot',
                                                   MoveRobotAction,
                                                   execute_cb=self._set_task,
                                                   auto_start=False)
        self._as.start()
        self.actionID = None
        rospy.Subscriber("depth", Float64, self._set_curr_depth)
        rospy.Subscriber("heading", Float64, self._set_curr_heading)
        self.rcchan = "rc"
        self.contolp = rospy.Publisher(self.rcchan, ManualControl, queue_size=10)
        self.pwm_center = 1500
        self.depth_p = 30
        self.depth_pmax = 300
        self.depth_tol = .1
        self.heading_p = 3
        self.heading_pmax = 300
        self.heading_tol = 2
        self.curr_depth = -1.
        self.curr_heading = -1.
        self.rate = rospy.Rate(10)


    def _set_task(self, goal):
        """Parse goal ID and send to correct handler"""
        self.actionID = goal.actionID
        if self.actionID == 'change_depth':
            self.change_depth(goal)
        elif self.actionID == 'change_heading':
            self.change_heading(goal)
        else:
            rospy.loginfo('%s actionID not recognized'%goal.actionID)
            self.actionID = None

    def change_depth(self, goal):
        """Proportional control to change depth"""
        target_depth = goal.target_depth
        ddiff = target_depth - self.curr_depth
        is_success = True

        while abs(ddiff) > self.depth_tol:
            # compute proportional controller output
            pout = ddiff * self.depth_p
            # limit output if necassary
            if abs(pout) > self.depth_pmax:
                if pout < 0:
                    pout = -self.depth_pmax
                else:
                    pout = self.depth_pmax
            if self._as.is_preempt_requested():
                rospy.loginfo('hello')
                self._as.set_preempted()
                is_success = False
                break
            pout += self.pwm_center
            # send command to RC channel
            h = Header(stamp=rospy.Time.now())
            controlout = ManualControl(header=h,
                                        x=self.pwm_center,
                                        y=self.pwm_center,
                                        z=pout,
                                        r=self.pwm_center)
            self.contolp.publish(controlout)

            # send command feedback
            self.send_feedback()
            self.rate.sleep()
            ddiff = target_depth - self.curr_depth

        if is_success:
            result = MoveRobotResult(actionID=goal.actionID,
                                     end_depth=target_depth)
            self._as.set_succeeded(result=result)


    def change_heading(self, goal):
        """Proportional control to change heading"""
        target_heading = goal.target_heading
        hdiff = target_heading - self.curr_heading
        is_success = True

        while abs(hdiff) > self.heading_tol:
            # compute proportional controller output
            pout = hdiff * self.heading_p
            # limit output if necassary
            if abs(pout) > self.heading_pmax:
                if pout < 0:
                    pout = -self.heading_pmax
                else:
                    pout = self.heading_pmax
            if self._as.is_preempt_requested():
                rospy.loginfo('hello')
                self._as.set_preempted()
                is_success = False
                break
            pout += self.pwm_center
            # send command to RC channel
            h = Header(stamp=rospy.Time.now())
            controlout = ManualControl(header=h,
                                        x=self.pwm_center,
                                        y=self.pwm_center,
                                        z=self.pwm_center,
                                        r=pout)
            self.contolp.publish(controlout)

            # send command feedback
            self.send_feedback()
            self.rate.sleep()
            hdiff = target_heading - self.curr_heading

        if is_success:
            result = MoveRobotResult(actionID=goal.actionID,
                                        end_heading=target_heading)
            self._as.set_succeeded(result=result)

    def send_feedback(self):
        """Send feedback about current robot state"""
        fb = MoveRobotFeedback(actionID=self.actionID,
                               current_depth=self.curr_depth,
                               current_heading=self.curr_heading)
        self._as.publish_feedback(fb)


    def _set_curr_depth(self, curr_depth):
        """Set the current depth when it is published"""
        self.curr_depth = curr_depth


    def _set_curr_heading(self, curr_heading):
        """Set the current depth when it is published"""
        self.curr_heading = curr_heading


if __name__ == '__main__':
    rospy.init_node('navigation_server')
    server = NavigationServer()
    rospy.spin()
