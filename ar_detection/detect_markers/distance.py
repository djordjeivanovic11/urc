import cv2
from cv2 import aruco
import numpy as np

data = np.load('camera_cal.npz')
mtx = data['mtx']
dist = data['dist']

MARKER_SIZE = 20 #cm
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters()
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break  
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, mtx, dist
        )
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv2.polylines(
                frame, [corners.astype(np.int32)], True, (0,255,255), 4, cv2.LINE_AA
            )
            corners = corners.reshape(4,2)
            corners = corners.astype(int)
            top_right = np.ravel(corners[0])
            top_left = np.ravel(corners[1])
            bottom_right = np.ravel(corners[2])
            bottom_left = np.ravel(corners[3])

            distance = np.sqrt(
                tvecs[i][0][0]**2 + tvecs[i][0][1]**2 + tvecs[i][0][2]**2
            )

            point = cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 4, 4)
            cv2.putText(
                frame,
                f"id: {ids[0]} Dist: {distance:.2f} cm",
                top_right,
                cv2.FONT_HERSHEY_PLAIN,
                1.3,
                (200,100,0),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                frame,
                f"x: {tvecs[i][0][0]:.2f} y: {tvecs[i][0][1]:.2f} z: {tvecs[i][0][2]:.2f}",
                bottom_right,
                cv2.FONT_HERSHEY_PLAIN,
                1.0,
                (0,0,255),
                2,
                cv2.LINE_AA,
            )
            x = tvecs[i][0][0]
            if (x < -30):
                direction = "far left"
            elif (x < -10):
                direction = "left"
            elif (x < 10):
                direction = "center"
            elif (x < 30):
                direction = "right"
            else:
                direction = "far right"
            print(f"Marker ID: {i}   Distance: {distance:.2f}   Direction: {direction}"), 

    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()