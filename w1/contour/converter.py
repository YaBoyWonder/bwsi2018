#!/usr/bin/env python

import cv2
import numpy as np


def nothing():
    pass

img = cv2.imread('orange.jpg')
#img = cv2.VideoCapture(0)
cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
cv2.namedWindow('properties')
cv2.createTrackbar('HLow', 'properties', 0, 255, nothing)
cv2.createTrackbar('SLow', 'properties', 0, 255, nothing)
cv2.createTrackbar('VLow', 'properties', 0, 255, nothing)
cv2.createTrackbar('HHigh', 'properties', 0, 255, nothing)
cv2.createTrackbar('SHigh', 'properties', 0, 255, nothing)
cv2.createTrackbar('VHigh', 'properties', 0, 255, nothing)


cv2.imshow('image', img)
while True:
    lower = np.array([cv2.getTrackbarPos('HLow', 'properties'), cv2.getTrackbarPos('SLow', 'properties'), cv2.getTrackbarPos('VLow', 'properties')])
    higher = np.array([cv2.getTrackbarPos('HHigh', 'properties'), cv2.getTrackbarPos('SHigh', 'properties'), cv2.getTrackbarPos('VHigh', 'properties')])
    newImg = cv2.inRange(img, lower, higher)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
        
        
    cv2.imshow('masked', newImg)
