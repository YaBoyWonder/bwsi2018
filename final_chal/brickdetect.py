#!/usr/bin/env python

'''
Github: YaBoyWonder
License: https://github.com/YaBoyWonder/Racecar/blob/master/LICENSE
'''
import rospy
import cv2
import math
import numpy as np

def nothing(xp): pass

if __name__ == '__main__':
    cv2.namedWindow('edit')
    cv2.createTrackbar('H min', 'edit', 0, 179, nothing)
    cv2.createTrackbar('H max', 'edit', 15, 179, nothing)
    cv2.createTrackbar('S min', 'edit', 110, 255, nothing)
    cv2.createTrackbar('S max', 'edit', 255, 255, nothing)
    cv2.createTrackbar('V min', 'edit', 123, 255, nothing)
    cv2.createTrackbar('V max', 'edit', 255, 255, nothing)

    img = cv2.imread('images/brick_wall.jpg', 1)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(img, (49,49), 0)
    blur_hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    while 1:
        hmin, smin, vmin = cv2.getTrackbarPos('H min', 'edit'), cv2.getTrackbarPos('S min', 'edit'), cv2.getTrackbarPos('V min', 'edit')
        hmax, smax, vmax =  cv2.getTrackbarPos('H max', 'edit'), cv2.getTrackbarPos('S max', 'edit'), cv2.getTrackbarPos('V max', 'edit')

        lower_range = np.array([hmin, smin, vmin])
        upper_range = np.array([hmax, smax, vmax])
        mask = cv2.inRange(blur_hsv, lower_range, upper_range)
        img2 = cv2.bitwise_and(blur_hsv, img_hsv, mask=mask)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_contour = None
        max_area = 0
        for c in contours:
            a = cv2.contourArea(c)
            if a > 20 and a > max_area:
                max_contour = c
                max_area = a
        x, y, w, h = cv2.boundingRect(max_contour)
        cv2.rectangle(img2, (x,y), (x+w, y+h), (0,0,255), 2)
        mix = cv2.addWeighted(blur, 0.5, img2, 1, 0)
        cv2.imshow('arrow', mix)

        k = cv2.waitKey(1) & 0xFF
        if k == 27: break
    exit()
