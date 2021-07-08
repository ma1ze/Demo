import cv2
import numpy as np
import pathlib
import cv2.aruco as aruco


#Calibration with chessboard-----------------------------------------------------------------
# def calibrate_chessboard(dir_path, image_format, square_size, width, height):
#     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#     # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
#     objp = np.zeros((height * width, 3), np.float32)
#     objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
#
#     objp = objp * square_size
#     # Arrays to store object points and image points from all the images.
#     objpoints = []  # 3d point in real world space
#     imgpoints = []  # 2d points in image plane.
#
#     images = pathlib.Path(dir_path).glob(f'*.{image_format}')
#     for fname in images:
#         img = cv2.imread(str(fname))
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         # Find the chess board corners
#         ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
#         # If found, add object points, image points (after refining them)
#         if ret:
#             objpoints.append(objp)
#
#             corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
#             imgpoints.append(corners2)
#     # Calibrate camera
#     ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
#     return [ret, mtx, dist, rvecs, tvecs]
#
# # Parameters
# IMAGES_DIR = 'path_to_images'
# IMAGES_FORMAT = '.jpg'
# SQUARE_SIZE = 1.6
# WIDTH = 6
# HEIGHT = 9

# Calibrate
# ret, mtx, dist, rvecs, tvecs = calibrate_chessboard(IMAGES_DIR, IMAGES_FORMAT,
#     SQUARE_SIZE,WIDTH, HEIGHT)

#undistort
# mtx, dist = load_coefficients('')
# original = cv2.imread('image.jpg')
# dst = cv2.undistort(original, mtx, dist, None, None)


#Calibration with ArUco---------------------------------------------------------------------
# aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
# # Dimensions in cm
# marker_length = 2.25
# marker_separation = 0.3
# arucoParams = aruco.DetectorParameters_create()
# board = aruco.GridBoard_create(5, 7, marker_length, marker_separation, aruco_dict)
#
# def calibrate_aruco(dirpath, image_format, marker_length, marker_separation):
#     aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
#     arucoParams = aruco.DetectorParameters_create()
#     board = aruco.GridBoard_create(5, 7, marker_length, marker_separation, aruco_dict)
#
#     counter, corners_list, id_list = [], [], []
#     img_dir = pathlib.Path(dirpath)
#     first = 0
#     # Find the ArUco markers inside each image
#     for img in img_dir.glob(f'*.{image_format}'):
#         image = cv2.imread(str(img))
#         img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         corners, ids, rejected = aruco.detectMarkers(
#             img_gray,
#             aruco_dict,
#             parameters=arucoParams
#         )
#
#         if first == 0:
#             corners_list = corners
#             id_list = ids
#         else:
#             corners_list = np.vstack((corners_list, corners))
#             id_list = np.vstack((id_list,ids))
#         first = first + 1
#         counter.append(len(ids))
#
#     counter = np.array(counter)
#     # Actual calibration
#     ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(
#         corners_list,
#         id_list,
#         counter,
#         board,
#         img_gray.shape,
#         None,
#         None
#     )
#     return [ret, mtx, dist, rvecs, tvecs]
#
# # Parameters
# IMAGES_DIR = 'path_to_images'
# IMAGES_FORMAT = '.jpg'
# # Dimensions in cm
# MARKER_LENGTH = 3
# MARKER_SEPARATION = 0.25
#
# # Calibrate
# ret, mtx, dist, rvecs, tvecs = calibrate_aruco(
#     IMAGES_DIR,
#     IMAGES_FORMAT,
#     MARKER_LENGTH,
#     MARKER_SEPARATION
# )

#ChArUco-----------------------------------------------------------------------------------------
def calibrate_charuco(dirpath, image_format, marker_length, square_length):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    board = aruco.CharucoBoard_create(7, 9, square_length, marker_length, aruco_dict)
    arucoParams = aruco.DetectorParameters_create()

    counter, corners_list, id_list = [], [], []
    img_dir = pathlib.Path(dirpath)
    first = 0
    # Find the ArUco markers inside each image
    for img in img_dir.glob(f'*{image_format}'):
        print(f'using image {img}')
        image = cv2.imread(str(img))
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(
            img_gray,
            aruco_dict,
            parameters=arucoParams
        )

        resp, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=img_gray,
            board=board
        )
        # If a Charuco board was found, let's collect image/corner points
        # Requiring at least 20 squares
        if resp > 10:
            # Add these corners and ids to our calibration arrays
            corners_list.append(charuco_corners)
            id_list.append(charuco_ids)

    # Actual calibration
    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(
        charucoCorners=corners_list,
        charucoIds=id_list,
        board=board,
        imageSize=img_gray.shape,
        cameraMatrix=None,
        distCoeffs=None)

    return [ret, mtx, dist, rvecs, tvecs]

# Parameters
IMAGES_DIR = '/home/dong/PycharmProjects/testDemo/images/calib'
IMAGES_FORMAT = 'png'
# Dimensions in cm
MARKER_LENGTH = 1.2
SQUARE_LENGTH = 2.35
# Calibrate
ret, mtx, dist, rvecs, tvecs = calibrate_charuco(
    IMAGES_DIR,
    IMAGES_FORMAT,
    MARKER_LENGTH,
    SQUARE_LENGTH

)
print("ret: "+str(ret))
print("Saving calibration data...")
calib_data = IMAGES_DIR + "/calib.yaml"
cv_file = cv2.FileStorage(calib_data, cv2.FILE_STORAGE_WRITE)
cv_file.write("camera_matrix", mtx)
cv_file.write("dist_coeff", dist)


