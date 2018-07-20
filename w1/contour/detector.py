#!/usr/bin/env python

import cv2
import numpy as np




#detecting contours around img

def detectContours(image):
	greyImage = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	edges = cv2.Canny(greyImage, (30, 200))
	_,contours,_ = cv2.findContours(edges,cv2.RETR_EXTERNAL)
	cv2.drawContours(image,contours,-1,(0,255,0),2)

	return image


	

#get the videocamera from videocapture

capture = cv2.VideoCapture(0)



#funcion code during while true to read and output images

while True:
	
	ret, frame = capture.read()
	greyImage = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
#	ret, frame = capture.read()
#	cv2.imshow('Orig.', bitwise_and.detectContours(frame))
#	cv2.imshow('Detection', bitwise_and.detectContours(frame))
	
	lower_range = np.array([0,172,172])
	upper_range = np.array([30,255,255])

	mask = cv2.inRange(greyImage, lower_range, upper_range)
	
	wisebit = cv2.bitwise_and(frame, frame,mask=mask)

	cv2.imshow('bitwise', wisebit)
	cv2.imshow('kappa', mask)



	if cv2.waitKey(1) == 13:
		break




capture.release()
cv2.destoryALlWindows()
