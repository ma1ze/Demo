import cv2
import numpy as np
import cv2.aruco as aruco
from aruco_image import *

import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import os
import time
import math
import matplotlib.pyplot as plt
from numpy import *

from angle_change import rotationVectorToEulerAngles

case =4

cap = cv2.VideoCapture(1)
if case == 3:
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 60)
if case == 1:
    cap.set(3, 1280)
    cap.set(4, 720)
    cap.set(5, 120)
if case ==2:
    cap.set(3, 640)
    cap.set(4, 360)
    cap.set(5, 330)
if case ==4:
    cap.set(3,1280)
    cap.set(4,720)
    cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(5,60)

# calib_loc = '/home/dong/PycharmProjects/testDemo/images/calib/calib_case1.yaml'
# calib_loc = '/home/dong/PycharmProjects/testDemo/images/calib/calib_case2.yaml'
# calib_loc = '/home/dong/PycharmProjects/testDemo/images/calib/calib_case3.yaml'
calib_loc = 'F:/python/webDemo/images/calib/calib.yaml'

cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)

mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()
font = cv2.FONT_HERSHEY_SIMPLEX

start_Time = time.perf_counter()
x = []
y = []
z = []
row = []
pitch = []
yaw = []
time_Now = []
timeFindConer = []
timeCaculation = []
totalFlash = 0
usefulFlash = 0

while (True):

    ret, frame = cap.read()
    if ret == True:

        result = aruco_image(frame)
        frame = result.getImg()





    else:
        pass

    # display the resulting frame
    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        end_Time = time.perf_counter()
        totalTime = end_Time - start_Time
        usefulFlashRate = str((usefulFlash/totalFlash)*100) + "%"

        break

# When everything done, release the capture
cap.release()

cv2.destroyAllWindows()



