# coding=UTF-8
from flask import Flask, render_template, Response
import cv2
import cv2.aruco as aruco
import numpy as np
from websocket_demo import returnCrossDomain
import socket
import time

calib_loc = 'F:/python/webDemo/images/calib/calib.yaml'

cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)

mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()
font = cv2.FONT_HERSHEY_SIMPLEX

class VideoCamera(object):
    def __init__(self):
        self.cap = cv2.VideoCapture(1)
        self.cap.set(3,1280)
        self.cap.set(4,720)
        self.cap.set(5,60)
        self.cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))

    def __del__(self):
        self.cap.release()

    def get_frame(self):
        success, image = self.cap.read()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        # aruco.CORNER_REFINE_CONTOUR
        parameters = aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
        parameters.adaptiveThreshConstant = 10

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if np.all(ids != None):
            rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
            for i in range(0, ids.size):
                # draw axis for the aruco markers
                aruco.drawAxis(image, mtx, dist, rvec[i], tvec[i], 0.1)
        ret, jpeg = cv2.imencode('.jpg', image)

        return jpeg.tobytes()


app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')


def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen(VideoCamera()), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='127.0.0.1', port=9999)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('127.0.0.1', 9999))
    sock.listen(5)
    while True:
        try:
            connection, address = sock.accept()
            returnCrossDomain(connection).start()
        except:
            time.sleep(1)



