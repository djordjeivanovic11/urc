import cv2 as cv
from cv2 import aruco
from pathlib import Path

# root directory of repo for relative path specification.
root = Path(__file__).parent.absolute()

# dictionary to specify type of the marker
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)

# MARKER_ID = 0
MARKER_SIZE = 400

# generating unique IDs using for loop
for id in range(20): # generate 20 markers
    # using function to draw a marker
    marker_image = aruco.generateImageMarker(marker_dict, id, MARKER_SIZE)
    cv.imshow("img", marker_image)
    cv.imwrite(f"{root}/marker_{id}.jpg", marker_image)