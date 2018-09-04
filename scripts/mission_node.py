#!/usr/bin/env python
"""
Mission node
============
Provides a framwork to build basic behaviors used to make up a mission.
"""

import roslib
roslib.load_manifest('zoidberg_nav')
import rospy
import time

class Mission:
    """Build up a mission with common task framework"""
    def __init__(self):
        """Setup possible tasks"""
        self.rate = rospy.Rate(10)

    def do(self, isterm_cb, timeout, guidance_cb=None):
        """callbacks provide termination condition and can modify guidance"""
        tstart = time.time()

        while not isterm_cb(self):
            # timeout termination behvior
            if time.time() - tstart > timout:
                rospy.loginfo('Timeout preempted')
                break

            if guidance_cb is not None:
                guidance_cb(self)

            # send command feedback
            self.rate.sleep()

    def _set_curr_vision(self, object_coords):
        """Set object x and y coordinates from vision"""
        self.object_x = object_coords.x_coord
        self.object_y = object_coords.y_coord
        self.object_width = object_coords.width
        self.object_depth = object_coords.depth

    def _set_curr_pose(self, dvl_output):
        """Set the currunt position, velocity and altitude from navigation"""
        pass


if __name__ == '__main__':
    rospy.init_node('mission')
    mission = Mission()
    rospy.spin()
