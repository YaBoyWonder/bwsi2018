from enum import Enum
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import LaserScan

from constants import *
from controllers import PIDController, PotentialFieldController


class State(Enum):

    follow_left = 0
    follow_right = 1
    potential_field = 2


class StateMachineNode:

    def __init__(self):
        rospy.init_node('state_machine')

        self.drive_publisher = rospy.Publisher(DRIVE_PUBLISHER_TOPIC, AckermannDriveStamped,
                                               queue_size=10)
        self.laser_subscriber = rospy.Subscriber(LASER_SUBSCRIBER_TOPIC, LaserScan,
                                                 self.laser_callback, queue_size=10)
        self.markers_subscriber = rospy.Subscriber(AR_TAG_SUBSCRIBER_TOPIC, AlvarMarkers,
                                                self.markers_callback,  queue_size=10)

        self.pf_controller = PotentialFieldController(gain=0.3)
        self.pid_controller = PIDController(kP=1.8)

        self.laser_data = []
        self.state = State.potential_field
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = 0.7
        self.goal_dist = 0.3

    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    def markers_callback(self, msg):
        self.check_state(msg.markers)
        print("State: %s" % self.state)

        if self.state == State.potential_field:
            self.drive_msg.drive.steering_angle = self.pf_controller.output(self.laser_data)
        else:
            end = int(0.7 * len(self.laser_data))
            perpendicular_dist = 0.0

            if self.state == State.follow_left:
                perpendicular_dist = min(self.laser_data[end:])
            elif self.state == State.follow_right:
                perpendicular_dist = min(self.laser_data[:end])
            
            print("Dist: %s" % perpendicular_dist)
            print("Error: %s" % (perpendicular_dist - self.goal_dist))
            self.drive_msg.drive.steering_angle = -self.pid_controller.output(perpendicular_dist,
                                                                             self.goal_dist)
        self.drive_publisher.publish(self.drive_msg)

    def check_state(self, markers):
        ids = [m.id for m in markers]
        if 23 in ids:
            self.state = State.potential_field
        elif 18 in ids or 22 in ids:
            self.state = State.follow_left
        elif 19 in ids:
            self.state = State.follow_right


if __name__ == '__main__':
    s = StateMachineNode()
    rospy.spin()
