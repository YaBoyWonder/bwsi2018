#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class safety:
    threshold = 1
    in_front = False
  def scan_callback(self, data):
        mid_dist = data.ranges[len(data.ranges)/2]
        print(mid_dist)
        if mid_dist < self.threshold:
            self.in_front = True
            #print("Obstruction detected")
        else:
            self.in_front = False
            #print("No obstruction detected")

    def drive_callback(self, drive_msg):
        print(drive_msg.drive.speed)
        if self.in_front:
            drive_msg.drive.speed = 0
        self.drive_pub.publish(drive_msg)

    def __init__(self):
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=5)
        self.drive_sub = rospy.Subscriber("/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped, self.drive_callback, queue_size=5)
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)


if __name__ == '__main__':
    rospy.init_node('safety', anonymous=True)
    node = safety()
    rospy.spin()
