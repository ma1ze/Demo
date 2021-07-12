import math
import numpy
import numpy as np
import cv2
import cv2.aruco as aruco
import time
import pybullet
import pybullet as p
import robotConf
import csv

index = 2
timeStep = 1. / 60.
gravity = 9.8
robotConf.setSimEnv(timeStep, gravity)

pos = [0, 0, 0]
rot = [0, 0, 0]

positionRobot = [0, 0, 0]
rotationRobot = [0, 0, 0]

# match the camera ID with your computer port ID (commonly 0/1/2)
cap = cv2.VideoCapture(0)
cap.set(3, 1080)
cap.set(4, 720)

calib_loc = '/home/frederick/Desktop/testDemo_add_UR10/images/calib/calib.yaml'
cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)
mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()
ids = []
prePositionRobot = [0, 0, 0]
prestate = [0, 0, 0]
ret, frame = cap.read()
dataFile = open("data.csv", "w")

while (True):
    # Capture single frame from camera
    ret, frame = cap.read()
    # robot.clawControl(1)
    if ret == True:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

    if np.all(ids != None):
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        # robot.clawOpen()
        for i in range(0, ids.size):
            # draw axis for the aruco markers
            aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)
        # draw a square around the markers
        aruco.drawDetectedMarkers(frame, corners)
        if ids[i][0] == 1:
            index = 1
        elif ids[i][0] == 2:
            index = 2
        else:
            index = 3

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

robot = robotConf.chooseRobot(index)

while (True):
    # Capture single frame from camera
    ret, frame = cap.read()
    # robot.clawControl(1)
    if ret == True:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

    if np.all(ids != None):
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        # robot.clawOpen()
        for i in range(0, ids.size):
            # draw axis for the aruco markers
            aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)
        # draw a square around the markers
        aruco.drawDetectedMarkers(frame, corners)
        robot.clawOpen()
        # if the aruco code #6 is captured by the camera, let the robot follow its track
        if ids[i][0] == 6:
            rotMatrixPosition = numpy.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
            positionRobot = numpy.dot(tvec[i][0], rotMatrixPosition)
            positionRobot = numpy.array(positionRobot) + robot.offset
            rotationCam = robotConf.rotationVectorToEulerAngles(rvec[i][0])
            rotationRobot = numpy.array([rotationCam[1], -rotationCam[0], -rotationCam[2] + math.pi / 2])
            if rotationRobot[2] >= math.pi:
                rotationRobot[2] = rotationRobot[2] - math.pi
        observeState, camState = robot.observeState(prestate)

        data = ["Camera", positionRobot[0], positionRobot[1], positionRobot[2],
                "Robot", observeState[0], observeState[1], observeState[2]]
        print(data)
        if len(ids) == 2:
            p.addUserDebugLine(prePositionRobot, positionRobot, lineColorRGB=[1, 0, 0], lifeTime=300, lineWidth=3)
            pybullet.addUserDebugLine([prestate[0], prestate[1], prestate[2]],
                                      [observeState[0], observeState[1], observeState[2]], lineColorRGB=[0, 1, 0],
                                      lifeTime=300, lineWidth=3)
            writer = csv.writer(dataFile)
            writer.writerow(data)
        # if cv2.waitKey(1) & 0xFF == ord('o'):
        #     robot.clawOpen()
        # print(positionRobot)
        prestate = observeState
        robot.arucoTracker(positionRobot, rotationRobot)
        robotConf.setCameraPicAndGetPic(robot)

        if cv2.waitKey(1) & 0xFF == ord('b'):
            robot.clawGrasp()

        prePositionRobot = positionRobot

    p.stepSimulation()
    time.sleep(timeStep)

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
