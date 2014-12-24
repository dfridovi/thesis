"""
Author: David Fridovich-Keil
Based on work for COS 429 final project, Fall 2014.

Use ORB keypoint matching to determine which pattern (if any) is present
in the first frame. Then use an IIR filter to guide matching on subsequent
frames, given the matching result from each previous frame. Also, for each
frame, recheck to see which template pattern is present, in case that changes.
If so, restart as though this were the first frame.
"""

import numpy as np
import scipy.misc as misc
import cv2
import time, sys, os
from filter2d import Filter2D

# file paths
EXPATH = "../data/templates/"
VIDEOPATH = "../data/orange_zebra/frames/"
VIDEOBASENAME = "orange_zebra%04d.jpg"
OUTPUTPATH = "../data/orange_zebra/frames_out_many_templates/"
OUTPUTBASENAME = "orange_zebra%04d_output.jpg"

# initialize global parameters
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
VALIDBOXAREATHRESH = 0.25
VALIDBOXDIMTHRESH = 50

# color filtering
def filterColor(img):

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

# read in all templates
# return a dictionary mapping filenames to filtered images, matchers, etc.
def readTemplates(filepath, orb):
    exfiles = [f for f in os.listdir(filepath) 
               if os.path.isfile(os.path.join(filepath,f))]
    templates = {}

    # loop through all template files
    for file in exfiles:

        # process example (see main loop for detailed comments)
        ex = cv2.imread(EXPATH + file)
        ex = cv2.resize(ex, (int(round(ex.shape[1]*SCALE)), 
                             int(round(ex.shape[0]*SCALE))))
        gray_ex = filterColor(ex)
        kp_ex, des_ex = orb.detectAndCompute(gray_ex, None)
        ex_h, ex_w = gray_ex.shape
        corners_ex = np.float32([[0, 0], [0, ex_h-1], [ex_w-1, ex_h-1], 
                                 [ex_w-1, 0]]).reshape(-1,1,2)

        # put the above into the 'file' entry of 'templates'
        templates[file] = {'gray' : gray_ex,
                           'kp' : kp_ex,
                           'des' : des_ex,
                           'corners' : corners_ex}
        
    # return the compiled dictionary
    return templates
    
# run matching on all templates and return best results
# ERROR HANDLING: if insufficient matches for all templates, return None
def findBestTemplate(templates, gray_test, orb, bf):

    # initialize dictionary to store matching parameters
    matching_params = {}

    # find ORB keypoints
    kp_test, des_test = orb.detectAndCompute(gray_test, None)

    if len(kp_test) == 0:
        return None, None

    # loop through all templates
    for exfile, params in templates.iteritems():
        
        # unpack params
        des_ex = params['des']
        
        # do feature matching
        matches = bf.knnMatch(des_ex, des_test, k=2)

        if (matches is not None) and (len(matches[0]) == 2):
                    
            # ratio test
            good_matches = []
            for m,n in matches:
                if m.distance < MATCHINGTHRESH * n.distance:
                    good_matches.append(m)
                
            # only add to dictionary if sufficient good matches
            if len(good_matches) >= MINGOODMATCHES:
                matching_params[exfile] = {'kp' : kp_test,
                                           'des' : des_test,
                                           'matches' : good_matches}
            
    # find best match (first check if any matches)
    if len(matching_params) == 0:
        return None, None

    best_match = None
    most_matches = -1
    for exfile, params in matching_params.iteritems():            
        num_matches = len(params['matches'])
        if num_matches > most_matches:
            most_matches = num_matches
            best_match = exfile
            
    return best_match, matching_params

# check if new bounding box is valid by ensuring the change in area is less
# than some threshold and that the smallest dimension is above 
# some threshold
def isValidBox(last_topleft, last_botright, new_topleft, new_botright):
    if ((last_topleft is not None) and (last_botright is not None) and
        (new_topleft is not None) and (new_botright is not None)):

        last_width = last_botright[0] - last_topleft[0] 
        last_height = last_botright[1] - last_topleft[1] 
        new_width = new_botright[0] - new_topleft[0] 
        new_height = new_botright[1] - new_topleft[1] 

        last_area = float(last_width * last_height)
        new_area = float(new_width * new_height)
        
        if ((abs(new_area - last_area) / last_area < VALIDBOXAREATHRESH) and 
            (min(new_width, new_height) > VALIDBOXDIMTHRESH)):
            return True
        return False

    return True

# run tracking
def runTracker():

    # initialize recurrent parameters
    last_topleft = None
    last_botright = None
    frame = 1
    
    # intialize ORB detector and BFMatcher
    orb = cv2.ORB()
    bf = cv2.BFMatcher(normType = cv2.NORM_HAMMING)

    # initialize time-series filter parameters -- one tracker per corner
    filter_topleft = None
    filter_botleft = None
    filter_botright = None
    filter_topright = None

    # get all templates
    templates = readTemplates(EXPATH, orb)

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
        gray_test = filterColor(test)
    
        # determine best example and get quality matches
        best_match, matching_params = findBestTemplate(templates, gray_test, 
                                                       orb, bf)
        
        # error handling
        if best_match is not None:

            # unpack parameters for test
            good_matches = matching_params[best_match]['matches']
            kp_test = matching_params[best_match]['kp']
            des_test = matching_params[best_match]['des']

            # unpack parameters for best template
            gray_ex = templates[best_match]['gray']
            kp_ex = templates[best_match]['kp']
            des_ex = templates[best_match]['des']
            corners_ex = templates[best_match]['corners']

            # estimate homography
            pts_ex = np.float32([kp_ex[m.queryIdx].pt 
                                 for m in good_matches]).reshape(-1,1,2)
            pts_test = np.float32([kp_test[m.trainIdx].pt 
                                   for m in good_matches]).reshape(-1,1,2)
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
        
            # check if valid box
            new_topleft = (offset[0] + min(
                    min(corners_test[0,0,1], corners_test[1,0,1]),
                    min(corners_test[2,0,1], corners_test[3,0,1])),
                            offset[1] + min(
                    min(corners_test[0,0,0], corners_test[1,0,0]),
                    min(corners_test[2,0,0], corners_test[3,0,0])))
            new_botright = (offset[0] + max(
                    max(corners_test[0,0,1], corners_test[1,0,1]),
                    max(corners_test[2,0,1], corners_test[3,0,1])),
                             offset[1] + max(
                    max(corners_test[0,0,0], corners_test[1,0,0]),
                    max(corners_test[2,0,0], corners_test[3,0,0])))

            if isValidBox(new_topleft, new_botright, 
                          last_topleft, last_botright):

                # update last corner coordinates
                last_topleft = new_topleft
                last_botright = new_botright

            else:
                print ("[FRAME " + str(frame) + 
                       "]: Found an invalid bounding box. " +  
                       "Using old bounding box.")
                corners_test = templates[best_match]['corners']

            # draw boundary of ex code
            cv2.polylines(gray_test, [np.int32(corners_test)], True, 120, 5)

            # save frame
            misc.imsave(OUTPUTPATH + OUTPUTBASENAME % frame, gray_test)

            # print elapsed time
            print ("Total elapsed time for frame " + str(frame) + ": " + 
                   str(time.time() - starttime) + " seconds")

            # update parameters (and repack) for next run
            templates[best_match]['kp'] = kp_test
            templates[best_match]['des'] = des_test
            templates[best_match]['corners'] = corners_test
            frame = frame + 1

        else:
            print "Did not find a good match in frame " + str(frame) + "."
            frame = frame + 1
            last_topleft = None
            last_topright = None
            offset = (0, 0)

if __name__ == "__main__":
   runTracker()
