import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import time
import math

cap = cv2.VideoCapture(1)
cap.set(3, 1280)
cap.set(4, 720)
cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
cap.set(5, 60)
calib_loc = 'F:/python/webDemo/images/calib/calib.yaml'
cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)

mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()
font = cv2.FONT_HERSHEY_SIMPLEX

start = time.perf_counter()
Distance_before = 0
Distance_after = 0
Time_before = 0
Time_after = 0
while (True):
    # Capture single frame from camera
    ret, frame = cap.read()
    if ret == True:

        time1 = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        time2 = time.perf_counter()
        # font for displaying text (below)
        Distance_before = Distance_after
        Time_before = Time_after


    if np.all(ids != None):
        # estimate pose of each marker and return the values
        # rvet and tvec-different from camera coefficients
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        #(rvec-tvec).any() # get rid of that nasty numpy value array error
        time3 = time.perf_counter()
        for i in range(0, ids.size):
            # draw axis for the aruco markers
            aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)

        # draw a square around the markers
        aruco.drawDetectedMarkers(frame, corners)


        # code to show ids of the marker found
        strg = ''
        for i in range(0, ids.size):
            strg += str(ids[i][0])+', '

        xPos = str(tvec[0][0][0])
        yPos = str(tvec[0][0][1])
        zPos = str(tvec[0][0][2])
        r1 = str(rvec[0][0][0])
        r2 = str(rvec[0][0][1])
        r3 = str(rvec[0][0][2])
        timeNow = time.perf_counter()-start
        distance = math.sqrt(math.pow(tvec[0][0][0],2)+math.pow(tvec[0][0][1],2)+math.pow(tvec[0][0][2],2))
        Distance_after = distance
        Time_after = timeNow
        velocity = abs((Distance_after-Distance_before)/(Time_after-Time_before))
        cv2.putText(frame, "Id: " + strg+"Xpos" + xPos+"  Ypos "+ yPos +" zPos "+zPos, (0, 64), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, "Distance: " + str(distance), (20, 120), font, 0.8, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, "Velocity: " + str(velocity), (20, 140), font, 0.8, (0, 255, 0), 1, cv2.LINE_AA)
        outputText =  "Time: "+str(timeNow)+" Id: " + strg+"Xpos:" + xPos+"  Ypos:"+ yPos +" zPos:"+zPos + " r1:"+r1 +" r2:"+r2+" r3:"+r3 +" Displace:"+str(distance) +" Velocity:"+str(velocity)
        print("timeSet: "+ str(time2-time1)+" TimeCornor"+str(time3-time2))
        # print(outputText)

    else:
        # code to show 'No Ids' when no markers are found
        cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

    # display the resulting frame
    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()

cv2.destroyAllWindows()

