"""
Analyze video feed. Find and mark faces.
"""

from SimpleCV import Image, Camera
from SimpleCV.Features import HaarCascade

HAAR_PATH = "/usr/local/share/OpenCV/haarcascades/"
HAAR_FACE_FILE = "haarcascade_frontalface_default.xml"
#HAAR_EYE_FILE = "haarcascade_eye.xml"

faces = HaarCascade(HAAR_PATH + HAAR_FACE_FILE)
#eyes = HaarCascade(HAAR_PATH + HAAR_EYE_FILE)

cam = Camera()

try:
    while True:
        img = cam.getImage()
        face_features = img.findHaarFeatures(faces)
 #       eye_features = img.findHaarFeatures(eyes)
        
        if face_features is not None:
            face_features.show()
 #       if eye_features is not None:
 #           eye_features.show()

except KeyboardInterrupt:
    print "\nKeyboardInterrupt registered. Terminating process."
