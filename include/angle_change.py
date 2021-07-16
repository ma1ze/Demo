#!/usr/bin/env python3
# coding:utf-8

import cv2
import numpy as np
import time
import threading
import os
import re
import subprocess
import random
import math


# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(angles1):
    theta = np.zeros((3, 1), dtype=np.float64)
    theta[0] = angles1[0] * 3.141592653589793 / 180.0
    theta[1] = angles1[1] * 3.141592653589793 / 180.0
    theta[2] = angles1[2] * 3.141592653589793 / 180.0
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_z, np.dot(R_y, R_x))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    # print('dst:', R)
    x = x * 180.0 / 3.141592653589793
    y = y * 180.0 / 3.141592653589793
    z = z * 180.0 / 3.141592653589793
    rvecstmp = np.zeros((1, 1, 3), dtype=np.float64)
    rvecs, _ = cv2.Rodrigues(R, rvecstmp)
    # print()
    return R, rvecs, x, y, z


def rotationVectorToEulerAngles(rvecs):
    R = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(rvecs, R)
    sy = math.sqrt(R[2, 1] * R[2, 1] + R[2, 2] * R[2, 2])
    sz = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    # print('sy=', sy, 'sz=', sz)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    # print('dst:', R)
    x = x * 180.0 / 3.141592653589793
    y = y * 180.0 / 3.141592653589793
    z = z * 180.0 / 3.141592653589793
    return x, y, z


if __name__ == '__main__':
    eulerAngles = np.zeros((3, 1), dtype=np.float64)
    eulerAngles[0] = 15.0
    eulerAngles[1] = 25.0
    eulerAngles[2] = 35.0
    R, rvecstmp, x, y, z = eulerAnglesToRotationMatrix(eulerAngles)
    print('输入欧拉角：\n', eulerAngles)
    print('旋转转矩：\n', R)
    # rvecstmp[0] = -2.100418
    # rvecstmp[1] = -2.167796
    # rvecstmp[2] = 0.273330 0.75467396][-0.00747155][ 0.00896453
    # rvecstmp[0] = 0.75467396
    # rvecstmp[1] = -0.00747155
    # rvecstmp[2] = 0.00896453

    print('旋转向量：\n', rvecstmp)
    print('计算后的欧拉角：\n', rotationMatrixToEulerAngles(rvecstmp))