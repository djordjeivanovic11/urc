import cv2
from cv2 import aruco
import numpy as np

# dictionary to specify type of the marker
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# initialize the paramaters for marker detection
param_markers = aruco.DetectorParameters()

# specify the video capture device by index (0 is default)
cap = cv2.VideoCapture(0)

# iterate through multiple frames, in a live video fee
while True:
    ret, frame = cap.read()
    if ret:
        # turning the frame to grayscale-only (for efficiency)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        # marker_corners = {[TL, TR, BR, BL], [TL, TR, BR, BL], [TL, TR, BR, BL]}
        # marker_IDs,    = { 0,                1,                2              }

        # getting corners of markers
        if marker_corners:
            for ids, corners in zip(marker_IDs, marker_corners):
                cv2.polylines(
                    frame, [corners.astype(np.int32)], True, (0,255,255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4,2)
                corners = corners.astype(int)
                top_right = np.ravel(corners[0])
                top_left = np.ravel(corners[1])
                bottom_right = np.ravel(corners[2])
                bottom_left = np.ravel(corners[3])
                cv2.putText(
                    frame,
                    f"id: {ids[0]}",
                    top_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (200,100,0),
                    2,
                    cv2.LINE_AA,
                )
        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key == ord("q"):
            break
cap.release()
cv2.destroyAllWindows()