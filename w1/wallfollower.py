def wallfollow():
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.speed = 2.0
    rad2 = 2**0.5

    # Read scan data, calc distance from wall + angle deviation, find new angle
    def scan_callback(data):
        # 90 degrees right: 180 units
        # 90 degrees left: 901 units
        # 45 degrees right: 362 units
        # 45 degrees left: 721 units

        p1 = data.ranges[180]
        constant = 1.6
        error = 1 - p1

        new_angle = constant*error
        drive_msg.drive.steering_angle = new_angle

    laser_sub = rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=10)
    drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        drive_pub.publish(drive_msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("wall_follower")
    wallfollow()
