import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import time
import math
import matplotlib.pyplot as plt
from numpy import *
from angle_change import rotationVectorToEulerAngles


class aruco_img_process:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.row = 0
        self.pitch = 0
        self.yaw = 0
        self.img_process = None

    def setX(self, x):
        self.x = x

    def setY(self, y):
        self.y = y

    def setZ(self, z):
        self.z = z

    def setRow(self, row):
        self.row = row

    def setPitch(self, pitch):
        self.pitch = pitch

    def setYaw(self, yaw):
        self.yaw = yaw

    def setImg(self, img):
        self.img_process = img

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getZ(self):
        return self.z

    def getRow(self):
        return self.row

    def getPitch(self):
        return self.pitch

    def getYaw(self):
        return self.yaw

    def getImg(self):
        return self.img_process


def aruco_image(img, contour_Refine = True ):
    result = aruco_img_process()
    calib_loc = 'F:/python/webDemo/images/calib/calib.yaml'

    cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)

    mtx = cv_file.getNode("camera_matrix").mat()
    dist = cv_file.getNode("dist_coeff").mat()
    font = cv2.FONT_HERSHEY_SIMPLEX
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    # aruco.CORNER_REFINE_CONTOUR
    parameters = aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
    parameters.adaptiveThreshConstant = 10

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if np.all(ids != None):
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        for i in range(0, ids.size):
            # draw axis for the aruco markers
            aruco.drawAxis(img, mtx, dist, rvec[i], tvec[i], 0.1)
            r1, r2, r3 = rotationVectorToEulerAngles(tvec[0][0])
            result.setX(tvec[0][0][0])
            result.setY(tvec[0][0][1])
            result.setZ(tvec[0][0][2])
            result.setRow(r1)
            result.setPitch(r2)
            result.setYaw(r3)
    aruco.drawDetectedMarkers(img, corners)

    result.setImg(img)

    return result
