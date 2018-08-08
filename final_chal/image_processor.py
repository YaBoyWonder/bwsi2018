#!/usr/bin/env python

'''
Github: YaBoyWonder
License: https://github.com/YaBoyWonder/Racecar/blob/master/LICENSE
'''

import rospy
import numpy as np
import cv2
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from constants import IMAGE_TOPIC

"""
    TODO: add processing for carwash
"""


class ImageProcessor:

    def __init__(self):
        self.img_subscriber = rospy.Subscriber(IMAGE_TOPIC, Image,
                                               self.img_callback, queue_size=10)

        self.lower_wall_range = np.array([0, 110, 123])
        self.upper_wall_range = np.array([15, 255, 218])

        self.img = []
        self.cv_bridge = CvBridge()

    def process_wall_img(self):
        img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(self.img, (49, 49), 0)
        blur_hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(blur_hsv, self.lower_wall_range, self.upper_wall_range)
        # img2 = cv2.bitwise_and(blur_hsv, img_hsv, mask=mask)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_contour = None
        max_area = 0
        for c in contours:
            a = cv2.contourArea(c)
            if a > 20 and a > max_area:
                max_contour = c
                max_area = a
                return True
        return False

    def process_arrow_img(self):
        img2 = np.copy(self.img)

        kernel = np.ones((5, 5), np.uint8)

        img = cv2.erode(self.img, kernel, iterations=1)
        img = cv2.dilate(self.img, kernel, iterations=1)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # cv2.inRange(img, np.array([0, 0, 0]), np.array([3, 7, 12]))

        edges = cv2.Canny(gray, 75, 150)

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, maxLineGap=1)
        angles = []
        goodLines = []
        bestLines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
                        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
            r, theta = self.cartesian_to_polar((x1, y1), (x2, y2))
            angles.append([r, theta, line])

        pairs = []
        for k1, v1 in enumerate(angles):
            if (v1[1] > 0 or v1[1] < 0) and (-55 < v1[1] < -25 or 25 < v1[1] < 55):
                pairs.append([v1[0], v1[1]])
                goodLines.append(v1[2])

        for line in goodLines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img2, (x1, y1), (x2, y2), (255, 0, 0), 2)
        try:
            for k, i in enumerate(goodLines):
                height, width = img.shape[:2]
                # print i[0][0]
                if i[0][0] > width / 2 and pairs[k][1]:
                    bestLines.append(1 * pairs[k][0] * (45 - 1 / (pairs[k][1])))
                else:
                    bestLines.append(-1 * pairs[k][0] * (45 - 1 / (pairs[k][1])))
        except:
            pass
        # print width
        # print np.mean(bestLines)
        if np.mean(bestLines) > 0:
            return 'l'
        else:
            return 'r'

    def cartesian_to_polar(self, (x1, y1), (x2, y2)):
        x = x2 - x1
        y = y2 - y1
        rho = np.sqrt(x ** 2 + y ** 2)
        phi = math.degrees(np.arctan2(y, x))
        return rho, phi

    def img_callback(self, msg):
        self.img = self.cv_bridge.imgmsg_to_cv2(msg)
  
