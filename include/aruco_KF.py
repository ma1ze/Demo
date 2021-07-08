from copy import deepcopy

import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import time
import math
from kalmanFilter import Kalman2D
import matplotlib.pyplot as plt

calib_loc = '/home/dong/PycharmProjects/testDemo/images/calib/calib.yaml'
cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)

mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()
font = cv2.FONT_HERSHEY_SIMPLEX

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)
cap.set(5, 120)
#
# calib_loc = '/home/dong/PycharmProjects/testDemo/images/calib/calib.yaml'
# cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)
#
# mtx = cv_file.getNode("camera_matrix").mat()
# dist = cv_file.getNode("dist_coeff").mat()
# font = cv2.FONT_HERSHEY_SIMPLEX

start = time.perf_counter()
# Distance_before = 0
# Distance_after = 0
# Time_before = 0
# Time_after = 0

measurePts1 =[]
measurePts2 =[]
measurePts3 =[]
measurePts4 =[]
kalmanPts1=[]
kalmanPts2=[]
kalmanPts3=[]
kalmanPts4=[]

kalman2dPt1 = Kalman2D()
kalman2dPt2 = Kalman2D()
kalman2dPt3 = Kalman2D()
kalman2dPt4 = Kalman2D()

Time = []
center_x = []
center_x_predicted = []
center_x_estimated = []
center_y = []
center_y_predicted = []
center_y_estimated = []
Distance = []
Distance_pre = []
Distance_est = []


while (True):
    # Capture single frame from camera
    ret, frame = cap.read()
    frame_kf_pre = frame.copy()
    frame_kf_esti = frame.copy()
    if ret == True:

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


    if np.all(ids != None):
        # print(corners[0])
        # print(corners[0][0][0][0])
        corners_KF_pre = deepcopy(corners)
        corners_KF_esti = deepcopy(corners)
        measure1 = (corners[0][0][0][0],corners[0][0][0][1])
        measure2 = (corners[0][0][1][0], corners[0][0][1][1])
        measure3 = (corners[0][0][2][0], corners[0][0][2][1])
        measure4 = (corners[0][0][3][0], corners[0][0][3][1])
        measurePts1.append(measure1)
        measurePts2.append(measure2)
        measurePts3.append(measure3)
        measurePts4.append(measure4)

        kalman2dPt1.update(corners[0][0][0][0], corners[0][0][0][1])
        kalman2dPt2.update(corners[0][0][1][0], corners[0][0][1][1])
        kalman2dPt3.update(corners[0][0][2][0], corners[0][0][2][1])
        kalman2dPt4.update(corners[0][0][3][0], corners[0][0][3][1])
        estimatedPt1 = [int(c1) for c1 in kalman2dPt1.getEstimate()]
        kalmanPts1.append(estimatedPt1)
        estimatedPt2 = [int(c2) for c2 in kalman2dPt2.getEstimate()]
        kalmanPts2.append(estimatedPt2)
        estimatedPt3 = [int(c3) for c3 in kalman2dPt3.getEstimate()]
        kalmanPts3.append(estimatedPt3)
        estimatedPt4 = [int(c4) for c4 in kalman2dPt4.getEstimate()]
        kalmanPts4.append(estimatedPt4)

        corners_KF_pre[0][0][0][0] = kalman2dPt1.predicted[0][0]
        corners_KF_pre[0][0][0][1] = kalman2dPt1.predicted[1][0]
        corners_KF_pre[0][0][1][0] = kalman2dPt2.predicted[0][0]
        corners_KF_pre[0][0][1][1] = kalman2dPt2.predicted[1][0]
        corners_KF_pre[0][0][2][0] = kalman2dPt3.predicted[0][0]
        corners_KF_pre[0][0][2][1] = kalman2dPt3.predicted[1][0]
        corners_KF_pre[0][0][3][0] = kalman2dPt4.predicted[0][0]
        corners_KF_pre[0][0][3][1] = kalman2dPt4.predicted[1][0]

        corners_KF_esti[0][0][0][0] = estimatedPt1[0]
        corners_KF_esti[0][0][0][1] = estimatedPt1[1]
        corners_KF_esti[0][0][1][0] = estimatedPt2[0]
        corners_KF_esti[0][0][1][1] = estimatedPt2[1]
        corners_KF_esti[0][0][2][0] = estimatedPt3[0]
        corners_KF_esti[0][0][2][1] = estimatedPt3[1]
        corners_KF_esti[0][0][3][0] = estimatedPt4[0]
        corners_KF_esti[0][0][3][1] = estimatedPt4[1]

        # print('cor')
        # print(corners)
        # print('pre')
        # print(corners_KF_pre)
        # print('etis')
        # print(corners_KF_esti)
        now = time.perf_counter()
        timeOff = now-start
        Time.append(timeOff)
        center_x_now = (corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0]+corners[0][0][3][0])/4
        center_x.append(center_x_now)
        center_y_now = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) / 4
        center_y.append(center_y_now)
        center_x_pre_now = (kalman2dPt1.predicted[0][0] + kalman2dPt2.predicted[0][0] + kalman2dPt3.predicted[0][0] + kalman2dPt4.predicted[0][0]) / 4
        center_x_predicted.append(center_x_pre_now)
        center_y_pre_now = (kalman2dPt1.predicted[1][0] + kalman2dPt2.predicted[1][0] + kalman2dPt3.predicted[1][0] + kalman2dPt4.predicted[1][0]) / 4
        center_y_predicted.append(center_y_pre_now)
        center_x_es_now = (estimatedPt1[0]+estimatedPt2[0]+estimatedPt3[0]+estimatedPt4[0])/4
        center_x_estimated.append(center_x_es_now)
        center_y_es_now = (estimatedPt1[1] + estimatedPt2[1] + estimatedPt3[1] + estimatedPt4[1]) / 4
        center_y_estimated.append(center_y_es_now)


        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawDetectedMarkers(frame_kf_pre, corners_KF_pre)
        aruco.drawDetectedMarkers(frame_kf_esti, corners_KF_esti)

        # estimate pose of each marker and return the values
        # rvet and tvec-different from camera coefficients
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        rvec_pre, tvec_pre , _ = aruco.estimatePoseSingleMarkers(corners_KF_pre,0.05,mtx,dist)
        rvec_est, tvec_est , _ = aruco.estimatePoseSingleMarkers(corners_KF_esti,0.05,mtx,dist)
        distance = math.sqrt(math.pow(tvec[0][0][0], 2) + math.pow(tvec[0][0][1], 2) + math.pow(tvec[0][0][2], 2))
        distance_pre = math.sqrt(math.pow(tvec_pre[0][0][0], 2) + math.pow(tvec_pre[0][0][1], 2) + math.pow(tvec_pre[0][0][2], 2))
        distance_est = math.sqrt(math.pow(tvec_est[0][0][0], 2) + math.pow(tvec_est[0][0][1], 2) + math.pow(tvec_est[0][0][2], 2))
        if distance_pre>1:
            distance_pre=0
        if distance_est>1:
            distance_est=0
        Distance.append(distance)
        Distance_pre.append(distance_pre)
        Distance_est.append(distance_est)
        cv2.putText(frame, "Distance: " + str(distance), (20, 120), font, 0.8, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame_kf_pre, "Distance: " + str(distance_pre), (20, 120), font, 0.8, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame_kf_esti, "Distance: " + str(distance_est), (20, 120), font, 0.8, (0, 255, 0), 1, cv2.LINE_AA)

    cv2.imshow('frame',frame)
    cv2.imshow('frame_Kf_pre',frame_kf_pre)
    cv2.imshow('frame_Kf_esti',frame_kf_esti)

    if cv2.waitKey(20) & 0xFF == ord('q'):
        plt.figure(1)
        plt.plot(Time,center_x,label= 'center_x_pix',color='r' )
        plt.plot(Time,center_x_predicted,label = 'center_x_predicted_pix',color = 'g')
        plt.plot(Time,center_x_estimated,label = 'center_x_estimated_pix',color='b')
        plt.show()
        plt.figure(2)
        plt.plot(Time,center_y,label= 'center_y_pix',color='r' )
        plt.plot(Time,center_y_predicted,label = 'center_y_predicted_pix',color = 'g')
        plt.plot(Time,center_y_estimated,label = 'center_y_estimated_pix',color='b')
        plt.show()

        plt.figure(3)
        plt.plot(Time, Distance, label='distance', color='r')
        plt.plot(Time, Distance_pre, label='distance_pre', color='g')
        plt.plot(Time, Distance_est, label='distance_est', color='b')
        plt.show()



        break

# When everything done, release the capture
cap.release()

cv2.destroyAllWindows()

