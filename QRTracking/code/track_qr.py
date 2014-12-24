"""
Author: David Fridovich-Keil
Based on work for COS 429 final project, Fall 2014.

Use Kalman filter to guide matching on the next frame, given the matching
result from the previous frame.
"""

import numpy as np
import scipy.misc as misc
import cv2
import time, sys, os
from filter2d import Filter2D

# file paths
IMGPATH = "../data/orange_zebra/"
EXFILENAME = "orange_zebra_template.jpg"
VIDEOPATH = "../data/orange_zebra/frames/"
VIDEOBASENAME = "orange_zebra%04d.jpg"
OUTPUTPATH = "../data/orange_zebra/frames_out/"
OUTPUTBASENAME = "orange_zebra%04d_output.jpg"

# initialize color filter parameters
UPPERBOUND_ORANGE = 25
LOWERBOUND_ORANGE = 110
UPPERBOUND_LUM = 200
LOWERBOUND_LUM = 75
MEDIANSIZE = 3
MATCHINGTHRESH = 0.6
MINGOODMATCHES = 4
SCALE = 0.5
CROPFACTOR = 1.2
FILTERTAP = 0.1

# initialize other recurrent parameters
last_topleft = None
last_botright = None
frame = 1

# color filtering
def filter_color(img):

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


# process example (see below for comments)
ex = cv2.imread(IMGPATH + EXFILENAME)
ex = cv2.resize(ex, (int(round(ex.shape[1]*SCALE)), 
                     int(round(ex.shape[0]*SCALE))))
gray_ex = filter_color(ex)
orb = cv2.ORB()
kp_ex, des_ex = orb.detectAndCompute(gray_ex, None)
bf = cv2.BFMatcher(normType = cv2.NORM_HAMMING)
ex_h, ex_w = gray_ex.shape
corners_ex = np.float32([[0, 0], [0, ex_h-1], 
                         [ex_w-1, ex_h-1], [ex_w-1, 0]]).reshape(-1,1,2)

# initialize time-series filter parameters -- one tracker per corner
filter_topleft = None
filter_botleft = None
filter_botright = None
filter_topright = None

# main loop
while os.path.isfile(VIDEOPATH + VIDEOBASENAME % frame):

    # start timer
    starttime = time.time()

    # read image and crop
    test_big = cv2.imread(VIDEOPATH + VIDEOBASENAME % frame)
    test_big = cv2.resize(test_big, (int(round(test_big.shape[1]*SCALE)), 
                                     int(round(test_big.shape[0]*SCALE))))
    test = test_big
    offset = (0, 0)
    if (last_topleft is not None) and (last_botright is not None):
        mid_row = 0.5 * (last_topleft[0] + last_botright[0])
        mid_col = 0.5 * (last_topleft[1] + last_botright[1])
        width = last_botright[0] - last_topleft[0]
        height = last_botright[1] - last_topleft[1]
        
        min_row = max(int(mid_row - CROPFACTOR * width/2.0), 0)
        max_row = min(int(mid_row + CROPFACTOR * width/2.0), 
                           test.shape[0])
        min_col = max(int(mid_col - CROPFACTOR * height/2.0), 0)
        max_col = min(int(mid_col + CROPFACTOR * height/2.0),
                           test.shape[1])
       
        offset = (min_row, min_col)
        test = test[min_row:max_row, min_col:max_col, :]

    # filter colors
    gray_test = filter_color(test)

    # find ORB keypoints
    kp_test, des_test = orb.detectAndCompute(gray_test, None)

    if len(kp_test) > 0:

        # do feature matching
        matches_ex = bf.knnMatch(des_ex, des_test, k=2)

        # ratio test
        good_matches_ex = []
        for m,n in matches_ex:
            if m.distance < MATCHINGTHRESH * n.distance:
                good_matches_ex.append(m)

        # halt if not enough good matches
        if len(good_matches_ex) < MINGOODMATCHES:
            print "Not enough good matches to estimate a homography."
            frame = frame + 1
            last_topleft = None
            last_topright = None
            offset = (0, 0)

        else:
    
            # estimate homography
            pts_ex = np.float32([kp_ex[m.queryIdx].pt 
                                 for m in good_matches_ex]).reshape(-1,1,2)
            pts_test = np.float32([kp_test[m.trainIdx].pt 
                                   for m in good_matches_ex]).reshape(-1,1,2)
            H, mask = cv2.findHomography(pts_ex, pts_test, cv2.RANSAC, 5.0)
            
            # use Filter filters to update corner positions
            # NOTE: to preserve units, we must transform back to original
            # (uncropped) coordinate system
            corners_test_raw = cv2.perspectiveTransform(corners_ex, H)
            obs_topleft = [corners_test_raw[0,0,0] + offset[1], 
                           corners_test_raw[0,0,1] + offset[0]]
            obs_botleft = [corners_test_raw[1,0,0] + offset[1], 
                           corners_test_raw[1,0,1] + offset[0]]
            obs_botright = [corners_test_raw[2,0,0] + offset[1], 
                            corners_test_raw[2,0,1] + offset[0]]
            obs_topright = [corners_test_raw[3,0,0] + offset[1], 
                            corners_test_raw[3,0,1] + offset[0]]

            # create new 2D filters if first run
            if frame == 1:
                initX_topleft = [obs_topleft[0], obs_topleft[1]] 
                initX_botleft = [obs_botleft[0], obs_botleft[1]]
                initX_botright = [obs_botright[0], obs_botright[1]]
                initX_topright = [obs_topright[0], obs_topright[1]]

                filter_topleft = Filter2D(initX_topleft, FILTERTAP)
                filter_botleft = Filter2D(initX_botleft, FILTERTAP)
                filter_botright = Filter2D(initX_botright, FILTERTAP)
                filter_topright = Filter2D(initX_topright, FILTERTAP)

            else:    
                filter_topleft.update(obs_topleft)
                filter_botleft.update(obs_botleft)
                filter_botright.update(obs_botright)
                filter_topright.update(obs_topright)

            # transform back to cropped coordinates
            filtered_topleft = filter_topleft.position()
            filtered_botleft = filter_botleft.position()
            filtered_botright = filter_botright.position()
            filtered_topright = filter_topright.position()
            
            corners_test = np.float32([[filtered_topleft[0] - offset[1],
                                        filtered_topleft[1] - offset[0]],
                                       [filtered_botleft[0] - offset[1],
                                        filtered_botleft[1] - offset[0]],
                                       [filtered_botright[0] - offset[1],
                                        filtered_botright[1] - offset[0]],
                                       [filtered_topright[0] - offset[1],
                                        filtered_topright[1] - offset[0]]]
                                      ).reshape(-1,1,2)

            # draw boundary of ex code
            cv2.polylines(gray_test, [np.int32(corners_test)], True, 120, 5)

            # update last corner coordinates
            last_topleft = (offset[0] + min(
                    min(corners_test[0,0,1], corners_test[1,0,1]),
                    min(corners_test[2,0,1], corners_test[3,0,1])),
                            offset[1] + min(
                    min(corners_test[0,0,0], corners_test[1,0,0]),
                    min(corners_test[2,0,0], corners_test[3,0,0])))
            last_botright = (offset[0] + max(
                    max(corners_test[0,0,1], corners_test[1,0,1]),
                    max(corners_test[2,0,1], corners_test[3,0,1])),
                             offset[1] + max(
                    max(corners_test[0,0,0], corners_test[1,0,0]),
                    max(corners_test[2,0,0], corners_test[3,0,0])))
            
            # print elapsed time
            print ("Total elapsed time for frame " + str(frame) + ": " + 
                   str(time.time() - starttime) + " seconds")

            # save frame
            misc.imsave(OUTPUTPATH + OUTPUTBASENAME % frame, gray_test)

            # update parameters for next run
            corners_ex = corners_test
            kp_ex = kp_test
            des_ex = des_test
            frame = frame + 1

    else:
        print "No keypoints found in frame " + str(frame) + "."
        frame = frame + 1
        last_topleft = None
        last_topright = None
        offset = (0, 0)
