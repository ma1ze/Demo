# -*- coding: utf-8 -*-
import cv2
import numpy as np

class Kalman2D(object):
    '''
    A class for 2D Kalman filtering
    '''

    def __init__(self, processNoiseCovariance=1e-4, measurementNoiseCovariance=1e-1, errorCovariancePost=0.1):

        # 状态空间：位置--2d,速度--2d,加速度--2d
        self.kalman = cv2.KalmanFilter(6, 2, 0)
        # self.kalman_state = cv.CreateMat(4, 1, cv.CV_32FC1)
        # self.kalman_state = np.mat(np.arange(4).reshape(4,1))
        # self.kalman_process_noise = np.mat(np.arange(4).reshape(4,1))
        self.measurement = np.array([[0],[0]],np.float32)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array(
            [[1, 0, 1, 0, 0.5, 0], [0, 1, 0, 1, 0, 0.5], [0, 0, 1, 0, 1, 0], [0, 0, 0, 1, 0, 1], [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], np.float32) * 0.003
        self.kalman.measurementNoiseCov = np.array([[0.2, 0], [0, 0.2]], np.float32) * 1

        self.predicted = None
        self.esitmated = None

    def update(self, x, y):
        '''
        Updates the filter with a new X,Y measurement
        '''

        self.measurement[0, 0] = x
        self.measurement[1, 0] = y

        self.predicted = self.kalman.predict()
        self.corrected = self.kalman.correct(self.measurement)

    def getEstimate(self):
        '''
        Returns the current X,Y estimate.
        '''

        return self.corrected[0,0], self.corrected[1,0]

    def getPrediction(self):
        '''
        Returns the current X,Y prediction.
        '''

        return self.predicted[0,0], self.predicted[1,0]


if __name__ == "__main__":
    kalman = Kalman2D()
