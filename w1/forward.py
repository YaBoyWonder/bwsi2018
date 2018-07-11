#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

def forward():
        # Create a node to publish information with
        rospy.init_node('forwardnode', anonymous=True)
        print("Initialized node")
        # Publish to the ackermann drive node
        pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0/', AckermannDriveStamped, queue_size=10)
        print("Initialized publisher")
        # Set the rate at which to send the message
        rate = rospy.Rate(5)
        drivemsg = AckermannDriveStamped()
        while not rospy.is_shutdown():
                drivemsg.drive.speed = 1.0
                print("Moving")
                # Publish information and sleep
                pub.publish(drivemsg)
                rate.sleep()

if __name__ == '__main__':
        try:
                forward()
        except rospy.ROSInterruptException:
                pass
