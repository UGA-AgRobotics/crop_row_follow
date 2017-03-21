import cv2
import numpy as np
from matplotlib import pyplot as plt

image = cv2.imread('../img/peanut_high.jpg', cv2.IMREAD_COLOR)
# blur = cv2.bilateralFilter(image, 15, 500, 500)
blur = cv2.GaussianBlur(image, (11, 11), 7)
b, g, r = cv2.split(blur)
I = 2 * g - r - b
open_k = np.ones((40, 40), np.uint8)
close_k = np.ones((60, 60), np.uint8)
closed = cv2.morphologyEx(I, cv2.MORPH_CLOSE, close_k)
opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, open_k)
# gblur = cv2.GaussianBlur(opened, (11, 11), 4)
# edges = cv2.Canny(gblur, 0, 50, L2gradient=True)
rt, th = cv2.threshold(opened, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
th = 255-th
# Hough wants my target to by 255
hough_image = np.matrix.copy(image)
lines = cv2.HoughLinesP(image=th, rho=1, theta=np.pi / 180, threshold=500, minLineLength=100, maxLineGap=100)
if lines is not None:
    for x in range(0, len(lines)):
        for x1, y1, x2, y2 in lines[x]:
            print x1, y1, x2, y2
            cv2.line(hough_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
else:
    print 'No lines'

# lines = cv2.HoughLines(th, 3, np.pi / 180, 50)
# if lines is not None:
#     for rho, theta in lines[0]:
#         a = np.cos(theta)
#         b = np.sin(theta)
#         x0 = a * rho
#         y0 = b * rho
#         x1 = int(x0 + 1000 * (-b))
#         y1 = int(y0 + 1000 * (a))
#         x2 = int(x0 - 1000 * (-b))
#         y2 = int(y0 - 1000 * (a))
#
#         print x1, y1, x2, y2
#
#         cv2.line(hough_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
# else:
#     print 'No lines'

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

plt.subplot(236)
plt.title('Lines')
plt.imshow(hough_image, cmap='gray')

plt.show()
