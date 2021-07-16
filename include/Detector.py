

import os
import sys

current_dir = os.path.dirname(__file__)
ailearn_dir = os.path.dirname(current_dir)
root_dir = os.path.dirname(ailearn_dir)
sys.path.append(current_dir)
sys.path.append(root_dir)

# default_config_dir = os.path.join(root_dir, "config", "detector.yaml")
default_config_dir ="F:/python/Demo/detector.yaml"
import cv2
import cv2.aruco as aruco
from tag import Tag
import numpy as np
import _thread
import yaml
from copy import deepcopy
from KalmanCornor import KalmanCornor
class Detector(object):

    def __init__(self, config_path=default_config_dir, aruco_dict=aruco.DICT_4X4_50, debug=False ,Kalman = False, Sharpen = False,Blur = False):
        self.config_path = config_path  # path of config file

        self._camera_id = self._readConfig('camera_id')
        self.camera = cv2.VideoCapture(self._camera_id)
        self.camera.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.tag_list = []
        self._frame_num = 3
        self.debug = debug
        self.Sharpen = Sharpen
        self.Blur = Blur
        self.Kalman = Kalman

        # Set camera parameters
        self._camera_width = self._readConfig("camera_width")
        self._camera_height = self._readConfig("camera_height")
        self._camera_fps = self._readConfig("camera_fps")
        # self._camera_type = self._readConfig("camera_type")
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._camera_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._camera_height)
        self.camera.set(5, self._camera_fps)

        self._marker_len = self._readConfig("marker_length")  # Length of marker in meters

        self._camera_matrix = np.array(self._readConfig("camera_matrix"))  # 3x3 camera intrinsic matrix
        self._camera_dist = np.array(self._readConfig("camera_dist"))  # vector of distortion coefficients
        self._aruco_dict = aruco.Dictionary_get(aruco_dict)
        self._parameters = aruco.DetectorParameters_create()
        if Sharpen:
            self._parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
            self._parameters.adaptiveThreshConstant = 10
        self._currentImage = None

        self.cornerFilter = KalmanCornor()


        # Thread control
        self._thread_running = False
        # self.startDetectionThread()

    def getTagList(self):
        '''Return the list of tags'''
        return self.tag_list

    def stopDetectionThread(self):
        '''Stop the detection thread from running constantly'''
        self._thread_running = False

    def startDetectionThread(self):
        '''Start running the detection thread constantly'''
        self._thread_running = True
        _thread.start_new_thread(self._detectionThread, ())

    def _detectionThread(self):
        '''Detection thread, used to constantly update tag list'''
        while self._thread_running:
            self._updateTagList()

    def _readConfig(self, keyword):
        '''Read a specific parameter from config file'''
        # TODO
        with open(self.config_path, 'r') as config:
            data = yaml.safe_load(config)
            return data[keyword]

    def _updateImage(self):
        ret, self._currentImage = self.camera.read()
        # cv2.imshow('read',self._currentImage)
        # cv2.waitKey(1)

        return ret

    def get_tag_list(self):
        # Process multiple frame, the final result is the
        # one with the largest number of tags detected
        corners = []
        if self.Kalman:
            cornersFilter = []

        ids = []
        for i in range(0, self._frame_num):
            ret = self._updateImage()
            if ret is False:
                print("******Fail to update the camera capture, please check the camera connection")
                return
            # Convert color image into gray
            if self.Blur:
                self._currentImage =cv2.GaussianBlur(self._currentImage, (11, 11), 0)

            gray = cv2.cvtColor(self._currentImage, cv2.COLOR_BGR2GRAY)

            # Get corner points and ids
            current_corners, current_ids, _ = aruco.detectMarkers(gray,
                                                                  self._aruco_dict,
                                                                  parameters=self._parameters)

            if current_ids is None:
                continue
            else:
                # If larger, or equal, update the list
                # The newest has the highest priority
                if len(current_ids) >= len(ids):
                    corners = current_corners

                    ids = current_ids

        # Simplify ids from [[id1],[id2],...] to [id1, id2,...]
        ids = [e[0] for e in ids]

        # Pick out board id and minor board id
        board_index = []
        board_ids = []
        board_corners = []

        # Delete major and minor board id from corners and ids
        board_index.sort()
        board_index.reverse()  # Reverse the list: from the higher to lower
        for index in board_index:
            del corners[index]
            del ids[index]
        # Pose estimation for the major board id tag
        for index, _id in enumerate(board_ids):
            if _id == self._board_id:  # Only fetch the major id to do estimation at the present
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers([board_corners[index]],
                                                                  self._board_id_len,
                                                                  self._camera_matrix,
                                                                  self._camera_dist)
                if self.Kalman:
                    self.cornerFilter.update(corners)
                    print('corners:')
                    print(corners)
                    corners = self.cornerFilter.getConrner_Filter()
                    print('filters:')
                    print(corners)
                # Matrix calculation
                mat_r, _ = cv2.Rodrigues(rvecs[0])  # Get 3x3 rotation matrix
                # Create a empty numpy array container
                mat_t = np.zeros([4, 4])
                mat_t[-1, :] = np.array([0, 0, 0, 1])  # Fill the last row with (0,0,0,1)
                mat_t[0:3, 0:3] = mat_r  # Fill the first 3 row&col with rotation matrix
                mat_t[0:3, -1] = tvecs[index]  # Fill the first 3 elements of the last col with translation vector
                offset_matrix = np.matrix([[1, 0, 0, 0],
                                           [0, 1, 0, self._board_offset],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])  # Offset transformation matrix in y axis
                mat_t = np.matmul(mat_t, offset_matrix)  # Apply offset translation
                mat_t_inv = np.linalg.inv(mat_t)  # Get the inverse of the matrix
                Tag.cali_t_mat = mat_t_inv  # Update transformation matrix for calibration


        # Pose estimation
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners,
                                                          self._marker_len,
                                                          self._camera_matrix,
                                                          self._camera_dist)



        # cv2.namedWindow("image")
        # cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
        if self.debug:
            image = deepcopy(self._currentImage)
            if ids is not None and len(ids) > 0:

                for index, rvec in enumerate(rvecs):
                    aruco.drawAxis(image, self._camera_matrix, self._camera_dist, rvec, tvecs[index], self._marker_len)
            aruco.drawDetectedMarkers(image, corners)
            # cv2.namedWindow("Debug",cv2.WINDOW_KEEPRATIO)

            cv2.imshow("Debug", image)
            cv2.waitKey(1)

        temp_tag_list = []

        for index, _id in enumerate(ids):

            temp_tag_list.append(Tag(_id, tvecs[index-1][0],rvecs[index-1][0]))

        # Update the tag list
        self.tag_list = temp_tag_list

        return temp_tag_list

def on_EVENT_LBUTTONDOWN(event, x, y, img, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        cv2.circle(img, (x, y), 1, (255, 0, 0), thickness=-1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (0, 0, 0), thickness=1)
        # cv2.imshow("image", img)
        return img

if __name__ == "__main__":
    detector = Detector(debug=True,Blur = False,Sharpen=False)
    id_count = np.zeros(50)
    while True:
        tag_list = detector.get_tag_list()
        for e in tag_list:
            id_count[e._id] += 1

    #     for tag in tag_list:
    #
    #
    #         print("ID: {}, x: {},y: {},z: {},row: {},pitch: {},yaw: {},time: {}".format(tag._id, tag.world_position[0], tag.world_position[1], tag.world_position[2],tag.Euler_angle[0],tag.Euler_angle[1],tag.Euler_angle[2],tag.time))
    #
    #     print("Next")
