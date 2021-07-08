from copy import deepcopy

import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import time
import math
from kalmanFilter import Kalman2D
import matplotlib.pyplot as plt

class KalmanCornor():
    def __init__(self):
        self.Kalman1 = Kalman2D()
        self.Kalman2 = Kalman2D()
        self.Kalman3 = Kalman2D()
        self.Kalman4 = Kalman2D()
        self.corner_Filter = []

    def update(self,corner):
        # print(corner[0][0][1])
        corners_KF_pre = deepcopy(corner)
        self.Kalman1.update(corner[0][0][0],corner[0][0][1])
        self.Kalman2.update(corner[0][1][0],corner[0][1][1])
        self.Kalman3.update(corner[0][2][0],corner[0][2][1])
        self.Kalman4.update(corner[0][3][0],corner[0][3][1])
        self.corner_Filter = deepcopy(corner)

    def getConrner_Filter(self):
        # self.corner_Filter.append(self.Kalman1.getEstimate())
        # self.corner_Filter.append(self.Kalman2.getEstimate())
        # self.corner_Filter.append(self.Kalman3.getEstimate())
        # self.corner_Filter.append(self.Kalman4.getEstimate())

        self.corner_Filter[0][0][0] = self.Kalman1.getEstimate()[0]
        self.corner_Filter[0][0][1] = self.Kalman1.getEstimate()[1]
        self.corner_Filter[0][1][0] = self.Kalman2.getEstimate()[0]
        self.corner_Filter[0][1][1] = self.Kalman2.getEstimate()[1]
        self.corner_Filter[0][2][0] = self.Kalman3.getEstimate()[0]
        self.corner_Filter[0][2][1] = self.Kalman3.getEstimate()[1]
        self.corner_Filter[0][3][0] = self.Kalman4.getEstimate()[0]
        self.corner_Filter[0][3][1] = self.Kalman4.getEstimate()[1]
        return self.corner_Filter