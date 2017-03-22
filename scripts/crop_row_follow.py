#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


def crop_line_image(image):
    OK = 20
    CK = 20
    hough_thresh = 700
    hough_min_line = 600
    hough_max_gap = 50

    # blur the image to get rid of some of that noise
    blur = cv2.GaussianBlur(image, (5, 5), 10)
    # break image into blue, green, red
    b, g, r = cv2.split(blur)
    # increase the amount of green relative to red and blue
    I = 2 * g - r - b
    # define the kernels for opening and closing
    open_k = np.ones((OK, OK), np.uint8)
    close_k = np.ones((CK, CK), np.uint8)
    # close the image to get rid of the small noise
    closed = cv2.morphologyEx(I, cv2.MORPH_CLOSE, close_k)
    # open to fill in any gaps
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, open_k)
    # threshold to binary with Otsu
    rt, th = cv2.threshold(opened, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # invert because Hough looks for 255
    th = 255 - th
    # deep copy the image for Hough
    hough_image = image
    # find the Hough lines using the PPHT method
    lines = cv2.HoughLinesP(image=th, rho=1, theta=np.pi / 100, threshold=hough_thresh, minLineLength=hough_min_line,
                            maxLineGap=hough_max_gap)
    if lines is not None:
        for x in range(0, len(lines)):
            count = 0
            for x1, y1, x2, y2 in lines[x]:
                angle = math.atan2(y1-y2, x1-x2) * (180 / math.pi)
                if 180 > angle > 0:
                    cv2.line(hough_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    count += 1
                else:
                    pass
        rospy.loginfo('Lines %d', len(lines))
    else:
        rospy.logwarn('No lines detected')

    return th


class ImgProc(object):
    def __init__(self):
        self.img_pub = rospy.Publisher('crop_rows', Image, queue_size=1)
        self.img_sub = rospy.Subscriber('camera/image_color/compressed', CompressedImage, self.crop_image_cb)
        self.bridge = CvBridge()

    def crop_image_cb(self, data):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(data)
            lines_img = crop_line_image(img)
            if lines_img is not None:
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(lines_img, 'mono8'))
            else:
                rospy.loginfo('No Image')
        except CvBridgeError as e:
            print e


def driver():
    rospy.init_node('crop_row_follow', anonymous=True)
    ImgProc()
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass
