#!/usr/bin/env python
"""
YaBoyWonder LICENSE: Apache-2.0
"""

import os
import rospy
from enum import Enum

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from controllers import PIDController


class WallFollowerNode:

    def __init__(self):
        rospy.init_node(STATE_MACHINE_NODE_NAME)

        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan,
                                                 self.laser_callback, queue_size=10)
        self.drive_publisher = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped,
                                               queue_size=10)

        self.steering_controller = PIDController(kP=1.2, kD=0)

        self.drive_msg = AckermannDriveStamped()
        self.cv_bridge = CvBridge()

        self.goal_distance = 0.8
        self.drive_speed = 1.2

    def laser_callback(self, msg):

        cur_dist = min(msg.ranges[int(0.4*len(msg.ranges)):int(0.6*len(msg.ranges))])
        front_dist = msg.ranges(len(msg.ranges)/2)

        steering_output = self.steering_controller.output(cur_dist, self.goal_distance)

        if front_dist < 2.0:
            self.drive_speed = 0.7
        else:
            self.drive_speed = 1.2

        self.drive_msg.drive.steering_angle = output
        self.drive_msg.drive.speed = self.drive_speed
        print("Distance from wall: %s" % cur_dist)                
        print("Error: %s" % (cur_dist - self.goal_distance))
        print("State: %s" % self.state)
        self.drive_publisher.publish(self.drive_msg)

if __name__ == '__main__':
    init()
    s = WallFollowerNode()
    rospy.spin()
