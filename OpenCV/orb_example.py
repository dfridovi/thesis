import numpy as np
import scipy.misc
import cv2
from matplotlib import pyplot as plt


img = cv2.imread('lena.jpg', 0)
img.shape

# Initiate STAR detector
orb = cv2.ORB()

# find the keypoints with ORB
kp = orb.detect(img)

# compute the descriptors with ORB
kp, des = orb.compute(img, kp)

# draw only keypoints location,not size and orientation
img = cv2.drawKeypoints(img, kp)
plt.imshow(img)
plt.show()
