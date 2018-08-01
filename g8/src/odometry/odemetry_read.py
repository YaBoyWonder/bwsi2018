#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

def dotprod((x1, y1), (x2, y2)):
    return x1*y1 + x2*y2

def mag(x, y):
    return math.sqrt(x*x + y*y)

def normal(x, y):
    m = mag(x, y)
    return (x/m, y/m) if m > 0 else (0, 0)

class OdometryRead:
    def __init__(self):
        rospy.init_node("odometryread")
        self.odom_sub = rospy.Subscriber("/vesc/odom", Odometry, self.odom_callback, queue_size=10)
        self.drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size = 10)
        self.drive_msg = AckermannDriveStamped()

        self.drive_msg.drive.speed = 0.5
        self.drive_msg.drive.steering_angle = 1.0

        self.goalX = 1.0
        self.goalY = 2.0

        self.rate = rospy.Rate(40)
        self.tol = 0.2
        while not rospy.is_shutdown():
            self.drive_pub.publish(self.drive_msg)
            self.rate.sleep() 

    def odom_callback(self, odom):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        px, py = p.x, p.y
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        print "%.5f %.5f" % (self.goalX - px, self.goalY - py)
        print "%.5f %.5f %.5f" % (roll, pitch, yaw)

        # Moving towards a point
        # Find vector pointing to goal vector
        dirX, dirY = normal(self.goalX - px, self.goalY - py)
        # Get normal vector
        norm = normal(px, py)
        # Get cosine of angle
        cosang = dotprod(norm, (dirX, dirY))
        # Get new steering angle
        self.drive_msg.drive.steering_angle = 0.5 * math.acos(cosang)

        if mag(dirX, dirY) <= self.tol:
            self.drive_msg.drive.speed = 0.0
            print "Done"

if __name__ == "__main__":
    o = OdometryRead()
    rospy.spin()
