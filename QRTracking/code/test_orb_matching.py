"""
Course: COS 429
Author: David Fridovich-Keil

Test feature matching using ORB features and RANSAC.
"""

import numpy as np
import random, math
import cv2
import drawMatches

import time, sys

# global parameters
MATCHINGTHRESH = 0.6
SCALE = 0.5

# file paths
IMGPATH = "../videos/orange_zebra/frames/"
TEMPPATH = "../images/"
TEMPFILENAME = "orange_zebra_template.jpg"
TESTFILENAME = "orange_zebra0001.jpg"

# color filtering
def filter_color(img):
    UPPERBOUND_ORANGE = 25
    LOWERBOUND_ORANGE = 110
    UPPERBOUND_LUM = 200
    LOWERBOUND_LUM = 75
    MEDIANSIZE = 3

    # convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    hue = hsv[:,:,0]

    # threshold
    gray = hsv[:,:,2]
    indices = np.logical_or(np.logical_and(hue > UPPERBOUND_ORANGE, 
                                           hue < LOWERBOUND_ORANGE),
                            gray < LOWERBOUND_LUM, 
                            gray > UPPERBOUND_LUM)
    gray[indices] = 255
    gray[np.logical_not(indices)] = 0
    gray = cv2.medianBlur(gray, MEDIANSIZE)

    return gray

# read images
test = cv2.imread(IMGPATH + TESTFILENAME)
test = cv2.resize(test, (int(round(test.shape[1]*SCALE)), 
                     int(round(test.shape[0]*SCALE))))
#test = test[100:600, 50:400, :] # for orange_chinese_2.JPG
#test = test[200:300, 200:400, :] # for orange_chinese_3.JPG

temp = cv2.imread(TEMPPATH + TEMPFILENAME)
temp = cv2.resize(temp, (int(round(temp.shape[1]*SCALE)), 
                     int(round(temp.shape[0]*SCALE))))

# create random composite
#random.seed()
#scale = random.uniform(SCALE, 2.0)
#scaled_temp = cv2.resize(temp, (int(round(scale*temp.shape[1])), 
#                            int(round(scale*temp.shape[0]))))
#paste_loc = (random.randint(0, test.shape[1] - scaled_temp.shape[1] - 1), 
#             random.randint(0, test.shape[0] - scaled_temp.shape[0] - 1))
#im = test
#im[paste_loc[1]:(paste_loc[1] + scaled_temp.shape[1]), 
#   paste_loc[0]:(paste_loc[0] + scaled_temp.shape[0])] = scaled_temp

# filter colors
gray_temp = filter_color(temp)
gray_test = filter_color(test)

# start timer
starttime = time.time()

# find ORB keypoints
orb = cv2.ORB()
kp_temp, des_temp = orb.detectAndCompute(gray_temp, None)
kp_test, des_test = orb.detectAndCompute(gray_test, None)

# do feature matching
bf = cv2.BFMatcher(normType = cv2.NORM_HAMMING)
matches_temp = bf.knnMatch(des_temp, des_test, k=2)

# ratio test
good_matches_temp = []
for m,n in matches_temp:
    if m.distance < MATCHINGTHRESH * n.distance:
        good_matches_temp.append(m)


# halt if not enough good matches
if len(good_matches_temp) < 4:
    print "Not enough good matches to estimate a homography."
    sys.exit()

# estimate homography
pts_temp = np.float32([kp_temp[m.queryIdx].pt 
                     for m in good_matches_temp]).reshape(-1,1,2)
pts_test = np.float32([kp_test[m.trainIdx].pt 
                     for m in good_matches_temp]).reshape(-1,1,2)
H_temp, mask_temp = cv2.findHomography(pts_temp, pts_test, cv2.RANSAC, 5.0)

# draw boundary of temp code
temp_h, temp_w = gray_temp.shape
corners_temp = np.float32([[0, 0], [0, temp_h-1], 
                         [temp_w-1, temp_h-1], [temp_w-1, 0]]).reshape(-1,1,2)
corners_test_temp = cv2.perspectiveTransform(corners_temp, H_temp)
cv2.polylines(gray_test, [np.int32(corners_test_temp)], True, 120, 5)

# filter out inliers
matchesMask_temp = mask_temp.ravel().tolist()
inliers = []
for i, m in enumerate(matchesMask_temp):
    if m == 1:
        inliers.append(good_matches_temp[i])

# print elapsed time
print "Total elapsed time: " + str(time.time() - starttime) + " seconds"

# show images
drawMatches.drawMatches(gray_temp, kp_temp, gray_test, kp_test, inliers) 
