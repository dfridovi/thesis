"""
Timing for video streaming.
"""

from SimpleCV import Image, Camera, JpegStreamCamera
import time

CAMERA_IP = "10.8.241.65"

cam = JpegStreamCamera("http://" + CAMERA_IP + "/video.mjpg")

try:
    while True:
        starttime = time.time()
        img = cam.getImage()
        print "Elapsed time: " + str(time.time() - starttime)
        img.show()

except KeyboardInterrupt:
    print "\nKeyboardInterrupt registered. Terminating process."
