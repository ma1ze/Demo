import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import time
import os
import math

cap = cv2.VideoCapture(2)

calib_loc = '/home/dong/PycharmProjects/testDemo/images/calib/calib.yaml'
cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)

mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()

font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
# cap.set(3, 1080)
# cap.set(4, 760)
# cap.set(5, 120)
# cap.set(10, 128)
# cap.set(11, 0)
# cap.set(12, 64)
# cap.set(13, 0)
# cap.set(15, 312)
# num = 0
while True:
    ret, frame = cap.read()

    w1, h1 = frame.shape[:2]
    # 读取摄像头画面
    # 纠正畸变
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h1, w1), 0, (h1, w1))
    dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    x, y, w1, h1 = roi
    dst1 = dst1[y:y + h1, x:x + w1]
    frame = dst1

    # 灰度化，检测aruco标签，所用字典为6×6——250
    if ret == True:

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()

    # 使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    #    如果找不打id
    if ids is not None:
        # 获取aruco返回的rvec旋转矩阵、tvec位移矩阵
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        # 估计每个标记的姿态并返回值rvet和tvec ---不同
        # rvec为旋转矩阵，tvec为位移矩阵
        # from camera coeficcients
        (rvec - tvec).any()  # get rid of that nasty numpy value array error
        # print(rvec)

        # 在画面上 标注auruco标签的各轴
        for i in range(rvec.shape[0]):
            aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
            aruco.drawDetectedMarkers(frame, corners, ids)

        ###### 显示id标记 #####
        cv2.putText(frame, "Id: " + str(ids), (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        ###### 角度估计 #####
        # print(rvec)
        # 考虑Z轴（蓝色）的角度
        # 本来正确的计算方式如下，但是由于蜜汁相机标定的问题，实测偏航角度能最大达到104°所以现在×90/104这个系数作为最终角度
        deg = rvec[0][0][2] / math.pi * 180
        # deg=rvec[0][0][2]/math.pi*180*90/104
        # 旋转矩阵到欧拉角
        R = np.zeros((3, 3), dtype=np.float64)
        cv2.Rodrigues(rvec, R)
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:  # 偏航，俯仰，滚动
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        # 偏航，俯仰，滚动换成角度
        rx = x * 180.0 / 3.141592653589793
        ry = y * 180.0 / 3.141592653589793
        rz = z * 180.0 / 3.141592653589793

        cv2.putText(frame, 'deg_z:' + str(ry) + str('deg'), (0, 140), font, 1, (0, 255, 0), 2,
                    cv2.LINE_AA)
        # print("偏航，俯仰，滚动",rx,ry,rz)

        ###### 距离估计 #####
        distance = ((tvec[0][0][2] + 0.02) * 0.0254) * 100  # 单位是米
        # distance = (tvec[0][0][2]) * 100  # 单位是米

        # 显示距离
        cv2.putText(frame, 'distance:' + str(round(distance, 4)) + str('m'), (0, 110), font, 1, (0, 255, 0), 2,
                    cv2.LINE_AA)

        ####真实坐标换算####（to do）
        # print('rvec:',rvec,'tvec:',tvec)
        # # new_tvec=np.array([[-0.01361995],[-0.01003278],[0.62165339]])
        # # 将相机坐标转换为真实坐标
        # r_matrix, d = cv2.Rodrigues(rvec)
        # r_matrix = -np.linalg.inv(r_matrix)  # 相机旋转矩阵
        # c_matrix = np.dot(r_matrix, tvec)  # 相机位置矩阵

    else:
        ##### DRAW "NO IDS" #####
        cv2.putText(frame, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # 显示结果画面
    cv2.imshow("frame", frame)

    key = cv2.waitKey(1)

    if key == 27:  # 按esc键退出
        print('esc break...')
        cap.release()
        cv2.destroyAllWindows()
        break

    if key == ord(' '):  # 按空格键保存
        #        num = num + 1
        #        filename = "frames_%s.jpg" % num  # 保存一张图像
        filename = str(time.time())[:10] + ".jpg"
        cv2.imwrite(filename, frame)

cap.release()

cv2.destroyAllWindows()

