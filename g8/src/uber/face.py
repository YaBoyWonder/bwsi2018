#!/usr/bin/env python

"""
YaBoyWonder LICENSE: Apache-2.0
"""

import cv2
import sys

face = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
video = cv2.VideoCapture(0)

while True:
    
    retval, frame = video.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    dem_faces = face_cascade.detectMultiScale(
        gray,
        scaleFactor-1.1,
        minNeighbors=5,
        minSize=(35, 35)
    )

    for (x, y, w, h) in dem_faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (50, 50, 200), 2)

    cv2.imshow('Faces', frame)

    if cv2.waitKey(1) == 13:
        sys.exit()

print("Made it here")

cv2.destroyAllWindows()
