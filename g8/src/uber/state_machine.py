#!/usr/bin/env python

"""
YaBoyWonder LICENSE: Apache-2.0
"""

import os
import rospy
from enum import Enum

from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

import cv2
import face_recognition
from controllers import PIDController

STATE_MACHINE_NODE_NAME = 'state_machine'
IMAGES_DIR = 'images'

faces_encodings = [None] * 8
ta_tag_dict = {
    'andrew.png': 1,
    'leo.png': 2,
    'mayank.png': 3,
    'karen.png': 4,
    'maryam.png': 5,
    'syed.png': 6,
    'carlos.png': 7,
    'bryan.png': 8
}


def init():
    print("Loading face encodings...")
    for filename in os.listdir(IMAGES_DIR):
        img = face_recognition.load_image_file(IMAGES_DIR + '/' + filename)
        face_encoding = face_recognition.face_encodings(img)
        faces_encodings[ta_tag_dict[filename]-1] = face_encoding
    print("Loaded face encodings.")

class State(Enum):
    search_for_face = 0
    search_for_tag = 1
    found_face = 2
    found_tag = 3
    turn_left = 4


class StateMachineNode:

    def __init__(self):
        rospy.init_node(STATE_MACHINE_NODE_NAME)

        self.ar_subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers,
                                              self.ar_callback, queue_size=10)
        self.img_subscriber = rospy.Subscriber('/zed/rgb/image_rect_color', Image,
                                               self.img_callback, queue_size=10)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan,
                                                 self.laser_callback, queue_size=10)
        self.drive_publisher = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped,
                                               queue_size=10)

        self.steering_controller = PIDController(kP=1.2, kD=0)

        self.drive_msg = AckermannDriveStamped()
        self.cv_bridge = CvBridge()

        self.goal_tag = -1
        self.goal_distance = 0.7
        self.drive_speed = 1.2
        self.ar_tag_threshold = 0.8

        self.is_found = False
        self.state = State.search_for_face

    def ar_callback(self, msg):
            if len(msg.markers) > 0:
                for tag in msg.markers:
                    id = tag.id
                    print("I see id %s" % id)
                    if id == self.goal_tag and self.state == State.search_for_tag\
                            and tag.pose.pose.position.z < self.ar_tag_threshold:
                        self.state = State.found_tag
                        self.is_found = True
                    elif self.is_found:
                        self.state = State.search_for_face
                        self.is_found = False
                    elif id == 9:
                        self.state = State.turn_left
            elif self.is_found:
                self.state = State.search_for_face
                self.is_found = False

    def img_callback(self, msg):
        global faces_encodings

        img_cv = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        rows, cols = img_cv.shape[0], img_cv.shape[1]
        img_left = img_cv[:, :int(0.9*cols)]

        # Collect results of face detection
        results = face_recognition.compare_faces(faces_encodings, img_left)

        distances = face_recognition.face_distance(faces_encodings, img_left)
        # Array containing the indices of found faces
        found_faces = [ i for i in range(len(results)) if results[i] == True]

        if len(found_faces) > 0 and self.state == State.search_for_face:
            print("I see faces")
            # List of tuples containing the distance AND the found index
            found_distances = [ (distances[found_faces[j]], j) for j in range(len(found_faces))]
            min_area = float('inf')
            index = 0
            for area, i in found_distances:
                if area < min_area:
                    min_area = area
                    index = i
            self.goal_tag = index + 1
            self.state = State.found_face
        # If it no longer sees the face yet it's still in the found state
        elif (self.goal_tag-1) not in found_faces and self.state == State.found_face:
            self.state = State.search_for_tag

    def laser_callback(self, msg):

        output = 0.0
        cur_dist = min(msg.ranges[int(0.4*len(msg.ranges)):int(0.85*len(msg.ranges))])

        if self.state == State.turn_left:
            output = -1.0
        else:
            output = -self.steering_controller.output(cur_dist, self.goal_distance)

        if self.state == State.search_for_face or self.state == State.found_tag:
            self.drive_speed = 1.2
        elif self.state == State.search_for_tag or self.state == State.found_face:
            self.drive_speed = -1.2

        self.drive_msg.drive.steering_angle = output
        self.drive_msg.drive.speed = self.drive_speed
        print("Distance from wall: %s" % cur_dist)                
        print("Error: %s" % (cur_dist - self.goal_distance))
        print("State: %s" % self.state)
        self.drive_publisher.publish(self.drive_msg)


if __name__ == '__main__':
    init()
    s = StateMachineNode()
    rospy.spin()
