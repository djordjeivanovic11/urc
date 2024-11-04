import cv2
from cv2 import aruco
import yaml
import numpy as np
from pathlib import Path
from tqdm import tqdm

# root directory of repo for relative path specification.
root = Path(__file__).parent.absolute()

# Set this flsg True for calibrating camera and False for validating results real time
calibrate_camera = True

# Set path to the images
calib_imgs_path = root.joinpath("board markers")

# For validating results, show aruco board to camera.
aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_6X6_1000 )

# Provide length (in cm) of the marker's side
markerLength = 3.75

# Provide separation between markers (in cm)
markerSeparation = 0.5

# Create ArUco board
board = aruco.GridBoard([4,5], markerLength, markerSeparation, aruco_dict)

arucoParams = aruco.DetectorParameters()

if calibrate_camera == True:
    img_list = []
    calib_fnms = calib_imgs_path.glob("*.jpg")
    print("Using...", end='')
    for idx, fn in enumerate(calib_fnms):
        print(idx, '', end='')
        img = cv2.imread(str(root.joinpath(fn)))
        cv2.imshow("img", img)
        img_list.append(img)
        h, w, c = img.shape
    print("Calibration images")

    counter, corners_list, id_list = [], [], []
    first = True
    for im in tqdm(img_list):
        img_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        cv2.imshow("img", img_gray)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(img_gray, aruco_dict, parameters=arucoParams)
        print(rejectedImgPoints, "debug")
        if first == True:
            corners_list = corners
            id_list = ids
            first = False
        else:
            corners_list = np.vstack((corners_list, corners))
            id_list = np.vstack((id_list, ids))
        counter.append(len(ids))
    print("Found {} unique markers".format(np.unique(ids)))

    counter = np.array(counter)
    print("Calibrating camera ... Please wait ...")

    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(corners_list, id_list, counter, board, img_gray.shape(), None, None)

    print("Camera matrix is \n", mtx, "\n And is stored in calibration.yaml file along with distortion coefficients : \n", dist)
    data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
    with open("calibration.yaml", "w") as f:
        yaml.dump(data, f)