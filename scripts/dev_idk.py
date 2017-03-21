import cv2
import numpy as np
from matplotlib import pyplot as plt

image = cv2.imread('../img/peanut_high.jpg', cv2.IMREAD_COLOR)
img_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
y, u, v = cv2.split(img_yuv)

u_blur = cv2.GaussianBlur(u, (41, 41), 0)

v_blur = cv2.GaussianBlur(v, (41, 41), 0)

rtu, u_thresh = cv2.threshold(u_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

rtv, v_thresh = cv2.threshold(v_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

img_and = cv2.bitwise_and(u_thresh, v_thresh)

edges = cv2.Canny(img_and, 10, 50, 2, L2gradient=True)

lines = cv2.HoughLines(edges, 1, np.pi / 180, 120)
for x in range(0, len(lines)):
    for rho, theta in lines[x]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))

        cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 10)

# minLineLength = 500
# maxLineGap = 500
# lines = cv2.HoughLinesP(img_and, 1, np.pi / 180, 1500, minLineLength, maxLineGap)
# for x in range(0, len(lines)):
#     for x1, y1, x2, y2 in lines[x]:
#         cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 10)
# plt.subplot(231)
# plt.title("U Gauss Blur")
# plt.imshow(u_blur, cmap='gray')
#
# plt.subplot(232)
# plt.title("V Gauss Blur")
# plt.imshow(v_blur, cmap='gray')
#
# plt.subplot(233)
# plt.title("U Otsu Threshold")
# plt.imshow(u_thresh, cmap='gray')
#
# plt.subplot(234)
# plt.title("V Otsu Threshold")
# plt.imshow(v_thresh, cmap='gray')

# plt.subplot(122)
plt.title("Lines")
plt.imshow(image)

# plt.subplot(121)
# plt.title("Edges")
# plt.imshow(edges, cmap='gray')

plt.show()




########################################################

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
