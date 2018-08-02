import cv2
import numpy as np

lower_range = np.array([70, 120, 2])
uppper_range = np.array([100, 240, 180])

img = cv2.imread('stoplight.png', 1)
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(img, lower_range, uppper_range)
img2 = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)

_, contours,_ = cv2.findContours(mask,  cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

for c in contours:
   area = cv2.contourArea(c)
   if area > 20:
       x, y, w, h = cv2.boundingRect(c)
       cv2.rectangle(img2, (x, y), (x + w, y + h), (0, 0, 255), 2)

cv2.imshow('green_light', img2)

cv2.waitKey(0)
exit()
