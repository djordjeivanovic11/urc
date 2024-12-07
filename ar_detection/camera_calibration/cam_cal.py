import numpy as np
import cv2, time


cv2.namedWindow("Image Feed")

cap = cv2.VideoCapture(0)

# setup camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

prev_frame_time = time.time()

cal_image_count = 0
frame_count = 0

while True:
    ret, frame = cap.read()
    
    # processing code
    frame_count += 1

    if frame_count == 30:
        cv2.imwrite("cal_image_" + str(cal_image_count) + ".jpg", frame)
        cal_image_count += 1
        frame_count = 0

    # calculate fps and display on frame
    new_frame_time = time.time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 40), cv2.FONT_HERSHEY_PLAIN, 1, (100, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("Image Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()