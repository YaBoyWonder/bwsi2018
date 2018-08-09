#!/usr/bin/env python

'''
Github: YaBoyWonder
License: https://github.com/YaBoyWonder/Racecar/blob/master/LICENSE
'''

import os
from enum import Enum
from threading import Thread
import rospy

from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Bool

from constants import *

from drive import Drive
from image_processor import ImageProcessor
from cv_bridge import CvBridge


class State(Enum):
    follow_left = 0
    follow_right = 1
    follow_middle = 2
    find_bobs_bricks = 3
    break_bobs_bricks = 4
    nothing = 5
    ptf = 6
    full_throttle = 7

class StateMachineNode:

    def __init__(self):
        rospy.init_node('state_machine')

        self.markers_subscriber = rospy.Subscriber(AR_TAG_TOPIC, AlvarMarkers,
                                                   self.markers_callback, queue_size=10)
        self.starter_subscriber = rospy.Subscriber(STARTER_TOPIC, Bool,
                                                   self.start_callback, queue_size=10)

        self.lights_off = False

        self.state = State.nothing

        self.drive = Drive()
        self.image_processor = ImageProcessor()
        self.cv_bridge = CvBridge()

        self.drive_speed = 1.2503309
        self.left_goal_dist = 0.90
        self.right_goal_dist = 0.8

    def markers_callback(self, msg):
        self.update_state(msg.markers)

        print("State: %s" % self.state)

        if self.state == State.follow_middle:
            self.drive.drive_middle(self.drive_speed)
                    elif self.state == State.follow_left:
            self.drive.drive_left(self.left_goal_dist, self.drive_speed)
        elif self.state == State.follow_right:
            self.drive.drive_right(self.right_goal_dist, self.drive_speed)
        elif self.state == State.ptf:
            if self.drive.is_safe():
                self.drive.drive_potential(speed=0.8)
            else:
                print("I am not safe. Correcting!")
                self.drive.drive_safety()
        elif self.state == State.full_throttle:
            self.drive.drive_right(self.right_goal_dist, 1.50)
        elif self.state == State.find_bobs_bricks:
            self.drive.drive_right(self.right_goal_dist, self.drive_speed)
            if self.image_processor.process_wall_img():
                self.state = State.break_bobs_bricks
        elif self.state == State.break_bobs_bricks:
            if self.drive.drive_straight(self.drive_speed, 1.0):
                self.state = State.follow_left

    def update_state(self, markers):
        ids = [m.id for m in markers]
        print("Tags: %s" % ids)
        if 12 in ids:
            self.drive.adjustment = 300
            self.drive_speed = 1.1
        else:
            self.drive.adjusment = 30
            self.drive.drive_speed = 1.25
        if 5 in ids:
            dist = markers[0].pose.pose.position.z
            if dist < 1.0:
                self.state = State.follow.middle

        if 13 in ids:
            self.drive.adjustment = 30
            self.drive_speed = 1.25
        print("adjust %s" % self.drive.adjustment)
        if 13 in ids:
            dist = markers[0].pose.pose.position.z
            print("Dist from marker: %s" % dist)
            if dist < 2.4:
                self.state = State.follow_middle
        if any([x in ids for x in FOLLOW_MIDDLE_TAGS]):
            self.state = State.follow_middle
        elif any([x in ids for x in FOLLOW_RIGHT_TAGS]):
            self.state = State.follow_right
        elif any([x in ids for x in FOLLOW_LEFT_TAGS]):
            self.state = State.follow_left
        elif any([x in ids for x in PTF_TAGS]) and markers[0].pose.pose.position.z < 2.5:
            self.state = State.ptf
        elif any([x in ids for x in FULL_THROTTLE_TAGS]):
            self.state = State.full_throttle
        elif self.lights_off and self.state == State.nothing:
            self.state = State.follow_middle

    def start_callback(self, msg):
        if msg.data:
            self.lights_off = True
            print("The lights are off.")


def launch_starter():
    os.system('python starter.py')


if __name__ == '__main__':
    t = Thread(target=launch_starter)
    t.start()
    s = StateMachineNode()
    rospy.spin()
