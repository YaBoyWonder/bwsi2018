#!/usr/bin/env python

'''
Github: YaBoyWonder
License: https://github.com/YaBoyWonder/Racecar/blob/master/LICENSE
'''

import rospy
import tf

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from controllers import PIDController, PotentialFieldController
from constants import DRIVE_TOPIC, LASER_TOPIC, ODOM_TOPIC


class Pose:

    def __init__(self, x=0.0, y=0.0, angle=0.0):
        self.x = x
        self.y = y
        self.angle = angle

    def __str__(self):
        return "X: %s" % self.x + " Y: %s" %  self.y + " Yaw: %s " % self.angle

class Drive:

    def __init__(self):
        self.drive_publisher = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped,
                                               queue_size=10)
        self.laser_subscriber = rospy.Subscriber(LASER_TOPIC, LaserScan,
                                                 self.laser_callback, queue_size=10)
        self.odom_subscriber = rospy.Subscriber(ODOM_TOPIC, Odometry,
                                                self.odom_callback, queue_size=10)


        self.middle_pid_controller = PIDController(kP=0.58)
        self.turn_pid_controller = PIDController(kP=0.92)
        self.angle_pid_controller = PIDController(kP=0.001)
        self.ptf_controller = PotentialFieldController(steer_gain=4.4, speed_gain=1.0, alpha=0.92, mu=0.06)

        self.drive_msg = AckermannDriveStamped()
        self.laser_data = []
        self.pose = None

        self.is_referenced = False
        self.reference_dist = 0.0
        self.reference_angle = 0.0

        self.safety_min_dist = 0.25
        self.adjustment= 30


    def drive_safety(self):
        speed = -0.7
        angle = 0.0
        if self.safety_check:
            angle = -1.0
        else:
        
    def safety_check(self):
        return self.laser_data[300] > self.laser_data[780]

    def is_safe(self):
        front_dist = self.laser_data[len(self.laser_data)/2]
        print("Front dist: %s " % front_dist)
        return front_dist >  self.safety_min_dist

    def drive_middle(self, speed):
        left_dist, right_dist = self.get_dists()
        angle = -self.middle_pid_controller.output(left_dist - right_dist, 0)
        print("Error: %s" % (left_dist - right_dist))
        self.drive(angle, speed)

    def drive_left(self, goal_dist, speed):
        left_dist, _ = self.get_dists()
        angle = -self.turn_pid_controller.output(left_dist, goal_dist)
        print("Error: %s" % (goal_dist - left_dist))
        self.drive(angle, speed)

    def drive_right(self, goal_dist, speed):
        _, right_dist = self.get_dists()
        angle = self.turn_pid_controller.output(right_dist, goal_dist)
        print("Error: %s " % (goal_dist - right_dist))
        self.drive(angle, speed)

    def drive_potential(self, speed):
        #laser_scan = self.laser_data[int(0.1*len(self.laser_data)):int(0.9*len(self.laser_data))]
        angle, new_speed = self.ptf_controller.output(self.laser_data)
        print("Angle: %s " % angle + " Speed: %s" % new_speed)
        self.drive(-angle, speed)

    def drive_straight(self, speed, dist):
        if self.pose is not None:
            if not self.is_referenced:
                self.reference_dist = self.pose.x
                self.reference_angle = self.pose.angle
                self.is_referenced = True
                print("Referecenced")

            pos_error = self.pose.x - self.reference_dist
            print("Change in dist: %s " % pos_error)

            if abs(pos_error) < dist:
                angular_output = self.angle_pid_controller.output(self.pose.angle, self.reference_angle)
                self.drive(angular_output, speed)
                return False

            self.is_referenced = False
            return True
        return False

    def drive(self, angle, speed):
        self.drive_msg.drive.speed = speed
        self.drive_msg.drive.steering_angle = angle
        print("Angle: %s " % angle + " Speed: %s" % speed)
        self.drive_publisher.publish(self.drive_msg)

    def get_dists(self):
        offset = len(self.laser_data) / 4
        adjustment = 0
        left_dist = min(self.laser_data[len(self.laser_data)-offset:len(self.laser_data)-offset+self.adjustment])
        right_dist = min(self.laser_data[offset:offset+self.adjustment])
        #left_dist = min(self.laser_data[int(0.5 * len(self.laser_data)):int(0.9 * len(self.laser_data))])
        #right_dist = min(self.laser_data[int(0.1 * len(self.laser_data)):int(0.5 * len(self.laser_data))])
        return left_dist, right_dist

    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        x, y = p.x, p.y
        _, _, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.pose = Pose(x=x, y=y, angle=yaw)


if __name__ == '__main__':
    rospy.init_node('drive_odom_test')
    d = Drive()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print(d.pose)
        if d.drive_straight(0.5, 1.0):
            print("Done")
            exit()
        rate.sleep()
            angle = 1.0
        self.drive(angle, speed)
