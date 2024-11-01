import cv2 as cv
from cv2 import aruco
import numpy as np

# dictionary to specify type of the marker
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)

# detect the marker
param_markers = aruco.DetectorParameters()

# utilizes the default camera/webcam driver
cap = cv.VideoCapture(0)

# iterate through multiple frames, in a live video fee
while True:
    ret, frame = cap.read()
    if ret:
        # turning the frame to grayscale-only (for efficiency)
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        # getting corners of markers
        if marker_corners:
            for ids, corners in zip(marker_IDs, marker_corners):
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0,255,255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4,2)
                corners = corners.astype(int)
                top_right = np.ravel(corners[0])
                top_left = np.ravel(corners[1])
                bottom_right = np.ravel(corners[2])
                bottom_left = np.ravel(corners[3])
                cv.putText(
                    frame,
                    f"id: {ids[0]}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (200,100,0),
                    2,
                    cv.LINE_AA,
                )
        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break
cap.release()
cv.destroyAllWindows()