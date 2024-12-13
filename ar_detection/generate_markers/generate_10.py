import cv2 as cv
from cv2 import aruco
#import os
#directory = r'C:\Users\caleb\URC\urc\ar_detection\generate_markers'
#os.chdir(directory)

# dictionary to specify type of the marker
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# MARKER_ID = 0
MARKER_SIZE = 400

# generating unique IDs using for loop
for id in range(10): # generate 20 markers
    # using function to draw a marker
    marker_image = aruco.generateImageMarker(marker_dict, id, MARKER_SIZE)
    cv.imshow("img", marker_image)
    cv.imwrite(f"markers/marker_{id}.png", marker_image)