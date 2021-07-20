import numpy as np
import cv2
import cv2.aruco as aruco
import time
cap = cv2.VideoCapture(1)

while (True):

    ret, frame = cap.read()
    if ret == True:

        time1 = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

        parameters = aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
        parameters.adaptiveThreshConstant = 10

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if np.all(ids != None):
        aruco.drawDetectedMarkers(frame, corners)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()