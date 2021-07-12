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
from aruco_image import *
import csv

# case =4

cap = cv2.VideoCapture(1)

cap.set(3,1280)
cap.set(4,720)
cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
cap.set(5,60)

calib_loc = 'F:/python/webDemo/images/calib/calib.yaml'

cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)

mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()
font = cv2.FONT_HERSHEY_SIMPLEX
number = 0
flag_record = False
time_record1 =0
time_record2 =0
start_Time = time.perf_counter()
x = []
y = []
z = []
row = []
pitch = []
yaw = []
f = open('文件名.csv','w',newline='')


csv_writer = csv.writer(f)

csv_writer.writerow(["mean_X","vari_X","var_X","mean_Y","vari_Y","var_Y","mean_Z","vari_Z","var_Z","mean_Row","vari_Row","var_Row","mean_Pitch","vari_Pitch","var_Pitch","mean_Yaw","vari_Yaw","var_Yaw"])


while (True):
    ret, frame = cap.read()
    if ret == True:
        result = aruco_img_process()

        time1 = time.perf_counter()
        blur = cv2.GaussianBlur(frame, (11, 11), 0)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

        parameters = aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
        parameters.adaptiveThreshConstant = 10

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        time2 = time.perf_counter()


    if np.all(ids != None):

        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        for i in range(0, ids.size):
            # draw axis for the aruco markers
            aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)
        if flag_record == True:
            x.append(tvec[0][0][0])
            y.append(tvec[0][0][1])
            z.append(tvec[0][0][2])
            # print(tvec[0][0])
            r1, r2, r3 = rotationVectorToEulerAngles(tvec[0][0])

            row.append(r1)
            pitch.append(r2)
            yaw.append(r3)
            if time.perf_counter()-time_record1>=3:
                flag_record = False

                mean_x = mean(x)*1000
                max_x = max(x)*1000
                min_x = min(x)*1000
                var_x = np.var(x)*1000
                if abs(max_x - mean_x) >= abs(mean_x - min_x):
                    vari_x = abs(max_x - mean_x)
                else:
                    vari_x = abs(mean_x - min_x)


                print("mean_X: " + str(mean_x))
                print("vari_X: " + str(vari_x))
                print("var_X" + str(var_x))

                # plt.show()



                mean_y = mean(y)*1000
                max_y = max(y)*1000
                min_y = min(y)*1000
                var_y = np.var(y)*1000
                if abs(max_y - mean_y) >= abs(mean_y - min_y):
                    vari_y = abs(max_y - mean_y)
                else:
                    vari_y = abs(mean_y - min_y)

                print("mean_Y: " + str(mean_y))
                print("vari_Y: " + str(vari_y))
                print("var_Y" + str(var_y))

                # plt.show()

                # plt.figure(3)
                # plt.scatter(time_Now, z, label='z', color='r')
                # plt.xlabel('time/s')
                # plt.ylabel('Z position/m')
                # plt.title('Z virange')
                # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/z.png')
                mean_z = mean(z)*1000
                max_z = max(z)*1000
                min_z = min(z)*1000
                var_z = np.var(z)*1000
                if abs(max_z - mean_z) >= abs(mean_z - min_z):
                    vari_z = abs(max_z - mean_z)
                else:
                    vari_z = abs(mean_z - min_z)

                print("mean_Z: " + str(mean_z))
                print("vari_Z: " + str(vari_z))
                print("var_Z" + str(var_z))

                # plt.show()

                # plt.figure(4)
                # plt.scatter(time_Now, row, label='row', color='r')
                # plt.xlabel('time/s')
                # plt.ylabel('Row/radian')
                # plt.title('Row virange')
                # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/Row.png')
                mean_row = mean(row)
                max_row = max(row)
                min_row = min(row)
                var_row = np.var(row)
                if abs(max_row - mean_row) >= abs(mean_row - min_row):
                    vari_row = abs(max_row - mean_row)
                else:
                    vari_row = abs(mean_row - min_row)

                print("mean_Row: " + str(mean_row))
                print("vari_Row: " + str(vari_row))
                print("var_Row" + str(var_row))

                # plt.show()
                #
                # plt.figure(5)
                # plt.scatter(time_Now, pitch, label='pitch', color='r')
                # plt.xlabel('time/s')
                # plt.ylabel('Pitch/radian')
                # plt.title('Pitch virange')
                # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/Pitch.png')
                mean_pitch = mean(pitch)
                max_pitch = max(pitch)
                min_pitch = min(pitch)
                var_pitch = np.var(pitch)
                if abs(max_pitch - mean_pitch) >= abs(mean_pitch - min_pitch):
                    vari_pitch = abs(max_pitch - mean_pitch)
                else:
                    vari_pitch = abs(mean_pitch - min_pitch)

                print("mean_Pitch: " + str(mean_pitch))
                print("vari_Pitch: " + str(vari_pitch))
                print("var_Pitch" + str(var_pitch))

                # plt.show()
                #
                # plt.figure(6)
                # plt.scatter(time_Now, yaw, label='', color='r')
                # plt.xlabel('time/s')
                # plt.ylabel('yaw/radian')
                # plt.title('yaw virange')
                # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/Yaw.png')
                mean_yaw = mean(yaw)
                max_yaw = max(yaw)
                min_yaw = min(yaw)
                var_yaw = np.var(yaw)
                if abs(max_yaw - mean_yaw) >= abs(mean_yaw - min_yaw):
                    vari_yaw = abs(max_yaw - mean_yaw)
                else:
                    vari_yaw = abs(mean_yaw - min_yaw)

                print("mean_Yaw: " + str(mean_yaw))
                print("vari_Yaw: " + str(vari_yaw))
                print("var_Yaw" + str(var_yaw))
                print("Finish the record....................")
                csv_writer.writerow([mean_x,vari_x,var_x,mean_y,vari_y,var_y,mean_z,vari_z,var_z,mean_row,vari_row,var_row,mean_pitch,vari_pitch,var_pitch,mean_yaw,vari_yaw,var_yaw])

    else:
        pass

    aruco.drawDetectedMarkers(frame, corners)

    cv2.imshow('frame',frame)


    if cv2.waitKey(1) & 0xFF == ord('s'):
        flag_record = True
        print("Start the record..........")
        x.clear()
        y.clear()
        z.clear()
        row.clear()
        pitch.clear()
        yaw.clear()
        time_record1 = time.perf_counter()
        number = number+1
        if number>=10:
            f.close()

    if cv2.waitKey(1) & 0xFF == ord('q'):

        break









