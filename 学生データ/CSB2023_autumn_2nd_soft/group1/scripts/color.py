#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ColorExtract(object):
    def __init__(self):
        self._image_pub = rospy.Publisher('masked_image', Image, queue_size=1)
        self._image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self._bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0,100,50])
        upper_red = np.array([20,255,255])

        color_mask = cv2.inRange(hsv, lower_red, upper_red)
        res = cv2.bitwise_and(cv_image, cv_image, mask=color_mask)
        try:
            self._image_pub.publish(self._bridge.cv2_to_imgmsg(res, 'bgr8'))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('color_extract')
    color = ColorExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
