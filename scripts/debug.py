import cv2
import numpy as np
from matplotlib import pyplot as plt


OK = 10
CK = 20
hough_thresh = 250
hough_min_line = 100
hough_max_gap = 50

col = 150
row = 580

image_raw = cv2.imread('../img/test1.png', cv2.IMREAD_COLOR)
image = image_raw[col:col+350, row:row+250]
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
hough_image = np.matrix.copy(image)
# find the Hough lines using the PPHT method
lines = cv2.HoughLinesP(image=th, rho=1, theta=np.pi / 180, threshold=hough_thresh, minLineLength=hough_min_line,
                        maxLineGap=hough_max_gap)
if lines is not None:
    for x in range(0, len(lines)):
        for x1, y1, x2, y2 in lines[x]:
            cv2.line(hough_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
else:
    print "No lines"

plt.subplot(231)
plt.title("Image")
plt.imshow(image, cmap='gray')

plt.subplot(232)
plt.title("Blur")
plt.imshow(blur, cmap='gray')

plt.subplot(233)
plt.title('I = g*2-r-b')
plt.imshow(I, cmap='gray')

plt.subplot(234)
plt.title('Close/Open')
plt.imshow(opened, cmap='gray')

plt.subplot(235)
plt.title('Otsu')
plt.imshow(th, cmap='gray')


image_raw[col:col+350, row:row+250] = hough_image
center = (image_raw.shape[1]/2, image_raw.shape[0]/2)
cv2.circle(image_raw, center, 50, (0, 255, 0), 5)
cv2.line(image_raw, pt1=(center[0], 0), pt2=(center[0], image_raw.shape[0]), thickness=3, color=(0, 255, 0))
cv2.line(image_raw, pt1=(0, center[1]), pt2=(image_raw.shape[1], center[1]), thickness=3, color=(0, 255, 0))
plt.subplot(236)
plt.title('Lines')
plt.imshow(image_raw, cmap='gray')

plt.show()
