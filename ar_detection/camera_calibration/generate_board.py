import cv2
from cv2 import aruco
import numpy as np
from pathlib import Path

# root directory of repo for relative path specification
root = Path(__file__).parent.absolute()

# Set this flag True for calibrating camera and False for validating results real time
calibrate_camera = True

# For validating results, show aruco board to camera
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)

# Provide length (in cm) of the marker's side
markerLength = 3.75

# Provide separation between markers (in cm)
markerSeparation = 0.5

# Create ArUco board
board = aruco.GridBoard([4,5], markerLength, markerSeparation, aruco_dict)

img = board.generateImage((864,1080))
cv2.imwrite(f"{root}/board.png", img)