#!/usr/bin/env python

import math
import numpy as np

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


class CropRowFind(object):
    def __init__(self):
        # TODO make class from type given by string
        rospy.loginfo('Starting crop row follow vision system...')
        self.vision = VisionCV2()
        self.rows = None

    def find_rows(self, data):
        self.rows = self.vision.find_rows(data)
        return self.rows

    def visual_debug(self, data):
        return self.vision.visual_debug(data)

    def draw_rows(self, img):
        img = img[self.vision.y1:self.vision.y2, self.vision.x1:self.vision.x2]
        if self.rows is not None and len(self.rows) > 1:
            for x1, y1, x2, y2 in self.rows:
                cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 10)
        return img


class VisionCV2(CropRowFind):
    def __init__(self):
        if rospy.has_param('roi'):
            window = rospy.get_param('roi')
            self.x1, self.x2, self.y1, self.y2 = window['x1'], window['x2'], window['y1'], window['y2']
        else:
            self.x1, self.y1, self.x2, self.y2 = 400, 400, 1000, 800
            rospy.set_param('roi', {'x1': self.x1, 'y1': self.y1, 'x2': self.x2, 'y2': self.y2})

        if rospy.has_param('gaussian_blur'):
            self.sigma = rospy.get_param('gaussian_blur/sigma')
            self.gauss_kernel = tuple(rospy.get_param('gaussian_blur/kernel'))
        else:
            self.sigma = 10
            self.gauss_kernel = (5, 5)
            rospy.set_param('gauss_blur/sigma', self.sigma)
            rospy.set_param('gauss_blur/kernel', list(self.gauss_kernel))

        if rospy.has_param('morph'):
            close_kernel = tuple(rospy.get_param('morph/close_kernel'))
            open_kernel = tuple(rospy.get_param('morph/open_kernel'))
            self.close_open_kernels = (close_kernel, open_kernel)
        else:
            self.close_open_kernels = ((20, 20), (10, 10))
            rospy.set_param('morph/close_kernel', list(self.close_open_kernels[0]))
            rospy.set_param('morph/open_kernel', list(self.close_open_kernels[1]))

        if rospy.has_param('hough'):
            self.hough_params = rospy.get_param('hough')
        else:
            self.hough_params = {'rho': 1, 'theta': 180, 'threshold': 250, 'minLineLength': 100, 'maxLineGap': 50,
                                 'maxAngle': 135, 'minAngle': 45}
            rospy.set_param('hough', self.hough_params)

        rospy.loginfo('roi: %s', str(rospy.get_param('roi')))
        rospy.loginfo('sigma: %s', str(self.sigma))
        rospy.loginfo('gaussian kernel: %s', str(self.gauss_kernel))
        rospy.loginfo('close and open kernel: %s', str(self.close_open_kernels))
        rospy.loginfo('hough parameter: %s', str(self.hough_params))

# TODO clean this up to be more ood
    def find_rows(self, data):
        img = self.roi(data)
        img = self.blur(img)
        img = self.egvi(img)
        img = self.close_open(img)
        rt, img = self.threshold(img)
        return self.lines(img)

    def visual_debug(self, data):
        debug_imgs = []
        debug_imgs.append(self.roi(data))
        debug_imgs.append(self.blur(debug_imgs[0]))
        debug_imgs.append(self.egvi(debug_imgs[1]))
        debug_imgs.append(self.close_open(debug_imgs[2]))
        debug_imgs.append(self.threshold(debug_imgs[3])[1])
        return debug_imgs

    def roi(self, img):
        return img[self.y1:self.y2, self.x1:self.x2]

    def blur(self, img):
        return cv2.GaussianBlur(img, self.gauss_kernel, self.sigma)

    def egvi(self, img):
        b, g, r = cv2.split(img)
        return 2 * g - r - b

    def close_open(self, img):
        closed = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones(self.close_open_kernels[0], np.uint8))
        return cv2.morphologyEx(closed, cv2.MORPH_OPEN, np.ones(self.close_open_kernels[1], np.uint8))

    def threshold(self, img):
        return cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    def lines(self, img):
        maxA = self.hough_params['maxAngle']
        minA = self.hough_params['minAngle']
        l_temp = cv2.HoughLinesP(img, self.hough_params['rho'], np.pi / self.hough_params['theta'],
                                 self.hough_params['threshold'], self.hough_params['minLineLength'],
                                 self.hough_params['maxLineGap'])
        l = []
        if l_temp is not None:
            for l_line in l_temp:
                for x1, y1, x2, y2 in l_line:
                    angle = int(math.atan2(y1 - y2, x1 - x2) * (180 / math.pi))
                    if maxA > angle > minA:
                        l.append([x1, y1, x2, y2])
                    else:
                        pass
        else:
            l = None
        return l


class CropRowFollow(object):
    def __init__(self):
        self.rows_img_pub = rospy.Publisher('rows_img', Image, queue_size=1)
        self.img_sub = rospy.Subscriber('camera/image_color/compressed', CompressedImage, self.crop_image_cb)
        self.bridge = CvBridge()
        self.crf = CropRowFind()

    def crop_image_cb(self, data):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(data)
            self.crf.find_rows(img)
            rows_img = self.crf.draw_rows(img)
            if rows_img is not None:
                self.rows_img_pub.publish(self.bridge.cv2_to_imgmsg(rows_img, 'bgr8'))
            else:
                rospy.loginfo('No Image')
        except CvBridgeError as e:
            print e


def main():
    rospy.init_node('crop_row_follow', anonymous=True)
    CropRowFollow()
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
