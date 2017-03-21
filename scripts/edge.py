import cv2
import numpy as np
from matplotlib import pyplot as plt

image = cv2.imread('../img/peanut_high.jpg', cv2.IMREAD_COLOR)
blur = cv2.bilateralFilter(image, 15, 500, 500)
b, g, r = cv2.split(blur)
I = 2 * g - r - b
open_k = np.ones((40, 40), np.uint8)
close_k = np.ones((60, 60), np.uint8)
closed = cv2.morphologyEx(I, cv2.MORPH_CLOSE, close_k)
opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, open_k)
gblur = cv2.GaussianBlur(opened, (11, 11), 4)
edges = cv2.Canny(closed, 100, 300, L2gradient=True)
lines = cv2.HoughLinesP(edges, 2, np.pi / 360, 50, 200, 10)
for x1, y1, x2, y2 in lines[0]:
    cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 5)

plt.title("Lines")
plt.imshow(image, cmap='gray')
plt.show()
