#!/usr/bin/env python

'''
Github: YaBoyWonder
License: https://github.com/YaBoyWonder/Racecar/blob/master/LICENSE
'''

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge

from std_msgs.msg import Bool
from sensor_msgs.msg import Image

from constants import STARTER_TOPIC, IMAGE_TOPIC, STARTER_NODE


class StarterNode:

    def __init__(self):
        rospy.init_node(STARTER_NODE)

        self.start_publisher = rospy.Publisher(STARTER_TOPIC, Bool,
                                               queue_size=10)

        self.img_subscriber = rospy.Subscriber(IMAGE_TOPIC, Image,
                                               self.img_callback, queue_size=10)

        self.cv_bridge = CvBridge()

        self.lower_range = np.array([39, 177, 76])
        self.upper_range = np.array([78, 255, 145])

    def img_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg)
        img = img[:720 / 2, :, :]
        blur = cv2.GaussianBlur(img, (25, 25), 0)
        blurhsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(blurhsv, self.lower_range, self.upper_range)
        img2 = cv2.bitwise_and(blurhsv, blurhsv, mask=mask)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_contour = None
        max_area = 0
        for c in contours:
            _, _, ew, eh = cv2.boundingRect(c)
            # Eccentricity
            e = ew / eh
            print("Eccentricity: %s" % e)

            area = cv2.contourArea(c)
            print("Area: %s " % area)
            if 0.95 < e < 1.05 and area > 30 and area > max_area:
                max_contour = c
                max_area = area
                self.start_publisher.publish(Bool(True))

                shutdown_mess = 'I see green. Go!'
                print(shutdown_mess)

                rospy.signal_shutdown(shutdown_mess)
                exit()

        print("I dont see go.")
        # x, y, w, h = cv2.boundingRect(max_contour)
        # cv2.rectangle(img2, (x, y), (x + w, y + h), (0, 0, 255), 2)
        #
        # mix = cv2.addWeighted(blur, 0.5, img2, 0.5, 0)


if __name__ == '__main__':
    s = StarterNode()
    rospy.spin()
