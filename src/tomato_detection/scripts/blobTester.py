import cv2
import numpy as np

img = cv2.imread("/home/tonello/Desktop/blob.png")
cv2.imshow("orig", img)
cv2.waitKey(0)


kernel_side = int(min(img.shape[0], img.shape[1]) / 3)  # height / 5
if kernel_side % 2 == 0:
    kernel_side += 1
    # The kernel for the gaussian filter has to be odd
print(kernel_side)
sigma = kernel_side / 6

gaussed = None
gaussed = cv2.GaussianBlur(img, (kernel_side, kernel_side), 0)
cv2.imshow("gaus", gaussed)
cv2.waitKey(0)


params = cv2.SimpleBlobDetector_Params()
params.filterByColor = 1
params.blobColor = 255

sbd = cv2.SimpleBlobDetector_create(params)
keypoints = sbd.detect(gaussed)

im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0, 0, 255),
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


cv2.imshow("Blobs", im_with_keypoints)
cv2.waitKey(0)


for keypoint in keypoints:
    print(keypoint)
