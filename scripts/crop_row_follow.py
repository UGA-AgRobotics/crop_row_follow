import cv2
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

# plt.subplot(121)
# plt.title("U & V")
# plt.imshow(img_and, cmap='gray')
#
# plt.subplot(122)
plt.title("Edges")
plt.imshow(edges, cmap='gray')

plt.show()
