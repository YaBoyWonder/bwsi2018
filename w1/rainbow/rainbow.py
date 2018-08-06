
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
from std_msgs.msg import String
import json


class LineFollowerNode:

    def __init__(self):
        self.cv_bridge = cv_bridge.CvBridge()
        self.min_area = 2000
        self.ranges = {
#            'blue': np.array([[100, 30, 30], [330, 255, 255]]),
            'orange': np.array([[0, 100, 160], [23, 255, 255]]),
 #           'yellow': np.array([[30, 100, 100], [40, 255, 255]]),
            'green': np.array([[75, 30, 30], [135, 255, 255]])
        }

        self.img_sub = rospy.Subscriber('/zed/rgb/image_rect_color', Image, self.img_callback,
                                        queue_size=10)
        self.new_img_pub = rospy.Publisher('/line', Image, queue_size=10)
        self.colors_pub = rospy.Publisher('/line_error', String, queue_size=10)

    def img_callback(self, img):
           cv_img = self.cv_bridge.imgmsg_to_cv2(img)

        colors = []

        rows, cols, _ = cv_img.shape
        cv_img = cv_img[int(0.9 * rows):, :]

        img_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        for color, range in self.ranges.items():

            mask = cv2.inRange(img_hsv, range[0], range[1])

            # img2 = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)

            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            max_contour, max_area = 0, 0
            for c in contours:
                area = cv2.contourArea(c)
                if area > max_area:
                    max_area = area
                    max_contour = c

            try:
                if max_area > self.min_area:
                    x, y, w, h = cv2.boundingRect(max_contour)
                    error = (x + w / 2) - cols / 2
                    print("I see %s" % color +  " with area %s" % area)
                    colors.append(color, error)
            except Exception as e:
                pass

            # img2 = cv2.rectangle(img2, (x, y), (x + w, y + h), (255, 255, 255), 5)

        self.colors_pub.publish(String(json.dumps(colors)))


if __name__ == '__main__':
    rospy.init_node('line_follower')
    l = LineFollowerNode()
    rospy.spin()
