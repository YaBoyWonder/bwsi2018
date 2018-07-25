#!usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
from std_msgs.msg import Float32


class LineFollowerNode:

    def __init__(self):
        self.cv_bridge = cv_bridge.CvBridge()

        self.lower_range = np.array([0, 100, 160])
        self.upper_range = np.array([23, 255, 255])

        self.img_sub = rospy.Subscriber('/zed/rgb/image_rect_color', Image, self.img_callback,
                                        queue_size=10)
        self.new_img_pub = rospy.Publisher('/line', Image, queue_size=10)
        self.error_pub = rospy.Publisher('/line_error', Float32, queue_size=10)

    def img_callback(self, img):
        cv_img = self.cv_bridge.imgmsg_to_cv2(img)

        rows, cols, _ = cv_img.shape
        cv_img = cv_img[int(0.9 * rows):, :]

        img_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(img_hsv, self.lower_range, self.upper_range)

        img2 = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        max_contour, max_area = 0, 0
        for c in contours:
            area = cv2.contourArea(c)
            if area > max_area:
                max_area = area
                max_contour = c

        x, y, w, h = cv2.boundingRect(max_contour)

        img2 = cv2.rectangle(img2, (x, y), (x + w, y + h), (255, 255, 255), 5)

        new_img = cv2.cvtColor(img2, cv2.COLOR_HSV2BGR)

        new_img = self.cv_bridge.cv2_to_imgmsg(new_img, 'bgr8')

        error = (x + w / 2) - cols / 2
        err_msg = Float32(error)

        self.error_pub.publish(err_msg)
        #self.new_img_pub.publish(new_img)


if __name__ == '__main__':
    rospy.init_node('line_follower')
    l = LineFollowerNode()
    rospy.spin()
