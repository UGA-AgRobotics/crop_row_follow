import cv2
import numpy as np
from matplotlib import pyplot as plt


class CropRowFind(object):
    def __init__(self):
        self.vision = VisionCV2()
        self.rows = None

    def find_rows(self, data):
        self.rows = self.vision.find_rows(data)
        return self.rows

    def draw_rows(self, img):
        if self.rows is not None:
            for x in range(0, len(self.rows)):
                for x1, y1, x2, y2 in self.rows[x]:
                    cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 10)
        else:
            print "No rows"
        return


class VisionCV2(CropRowFind):
    def __init__(self):
        self.window = (150, 250, 580, 680)
        self.sigma = 10
        self.gauss_kernel = (5, 5)
        self.close_open_kernels = ((20, 20), (10, 10))
        self.hough_params = (1, 180, 250, 100, 50)

    def find_rows(self, data):
        img = np.matrix.copy(data)
        self.roi(img)
        self.blur(img)
        self.egvi(img)
        self.close_open(img)
        self.threshold(img)
        return self.lines(img)

    def roi(self, img):
        return img[self.window[0]:self.window[1], self.window[2]:self.window[3]]

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
        return cv2.HoughLinesP(image=img, rho=self.hough_params[0], theta=np.pi / self.hough_params[1],
                               threshold=self.hough_params[2], minLineLength=self.hough_params[3],
                               maxLineGap=self.hough_params[4])


# image = image_raw[col:col + 350, row:row + 250]
# # blur the image to get rid of some of that noise
# blur = cv2.GaussianBlur(image, (5, 5), 10)
# # break image into blue, green, red
# b, g, r = cv2.split(blur)
# # increase the amount of green relative to red and blue
# I = 2 * g - r - b
# # define the kernels for opening and closing
# open_k = np.ones((OK, OK), np.uint8)
# close_k = np.ones((CK, CK), np.uint8)
# # close the image to get rid of the small noise
# closed = cv2.morphologyEx(I, cv2.MORPH_CLOSE, close_k)
# # open to fill in any gaps
# opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, open_k)
# # threshold to binary with Otsu
# rt, th = cv2.threshold(opened, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# # invert because Hough looks for 255
# th = 255 - th
# # deep copy the image for Hough
# hough_image = np.matrix.copy(image)
# # find the Hough lines using the PPHT method
# lines = cv2.HoughLinesP(image=th, rho=1, theta=np.pi / 180, threshold=hough_thresh, minLineLength=hough_min_line,
#                         maxLineGap=hough_max_gap)


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
# plt.title('Otsu')
# plt.imshow(th, cmap='gray')
#
# image_raw[col:col + 350, row:row + 250] = hough_image
# center = (image_raw.shape[1] / 2, image_raw.shape[0] / 2)
# cv2.circle(image_raw, center, 50, (0, 255, 0), 5)
# cv2.line(image_raw, pt1=(center[0], 0), pt2=(center[0], image_raw.shape[0]), thickness=3, color=(0, 255, 0))
# cv2.line(image_raw, pt1=(0, center[1]), pt2=(image_raw.shape[1], center[1]), thickness=3, color=(0, 255, 0))
# plt.subplot(236)
# plt.title('Lines')
# plt.imshow(image_raw, cmap='gray')
#
# plt.show()

crf = CropRowFind()
image = cv2.imread('../img/test1.png', cv2.IMREAD_COLOR)
crf.find_rows(image)
plt.imshow(crf.draw_rows(image))
