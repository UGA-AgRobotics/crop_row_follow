#!/usr/bin/python

import cv2
import numpy as np
import rospy


def vison_stuff():
    image = cv2.imread('../img/peanut_high.jpg', cv2.IMREAD_COLOR)
    # blur the image to get rid of some of that noise
    blur = cv2.GaussianBlur(image, (5, 5), 7)
    # break image into blue, green, red
    b, g, r = cv2.split(blur)
    # increase the amount of green relative to red and blue
    I = 2 * g - r - b
    # define the kernels for opening and closing
    open_k = np.ones((40, 40), np.uint8)
    close_k = np.ones((60, 60), np.uint8)
    # close the image to get rid of the small noise
    closed = cv2.morphologyEx(I, cv2.MORPH_CLOSE, close_k)
    # open to fill in any gaps
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, open_k)
    # threshold to binary with Otsu
    rt, th = cv2.threshold(opened, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # invert because Hough looks for 255
    th = 255-th
    # deep copy the image for Hough
    hough_image = np.matrix.copy(image)
    # find the Hough lines using the PPHT method
    lines = cv2.HoughLinesP(image=th, rho=1, theta=np.pi / 180, threshold=2500, minLineLength=600, maxLineGap=200)
    if lines is not None:
        for x in range(0, len(lines)):
            for x1, y1, x2, y2 in lines[x]:
                print x1, y1, x2, y2
                cv2.line(hough_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    else:
        print 'No lines'


    # plt.subplot(231)
    # plt.title("Image")
    # plt.imshow(image, cmap='gray')
    #
    # plt.subplot(232)
    # plt.title("Blur")
    # plt.imshow(blur, cmap='gray')
    #
    # plt.subplot(233)
    # plt.title('I = g*2-r-b')
    # plt.imshow(I, cmap='gray')
    #
    # plt.subplot(234)
    # plt.title('Close/Open')
    # plt.imshow(opened, cmap='gray')
    #
    # plt.subplot(235)
    # plt.title('Otsu inverted')
    # plt.imshow(th, cmap='gray')
    #
    # plt.subplot(236)
    # plt.title('Lines')
    # plt.imshow(hough_image, cmap='gray')
    #
    # plt.show()

def test():
    rospy.init_node('crop_row_follow', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print 'test'
        rospy.sleep()


if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
