import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import time
import os

def loadAugImages(path):
    mylist = os.listdir(path)
    numOfMalers = len(mylist)
    print("Total number maekers detected", numOfMalers)
    augDics = {}
    for imgPath in mylist:
        key = int(os.path.splitext(imgPath)[0])
        imgAug = cv2.imread(f"{path}/{imgPath}")
        augDics[key] = imgAug

    return augDics


def findArucoMarker(img,maekerSize = 4, totalMarkers =50, draw =True):
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    key = getattr(aruco,f'DICT_{maekerSize}X{maekerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(imgGray,arucoDict,parameters=arucoParam)
    # print(ids)

    if draw:
        aruco.drawDetectedMarkers(img,bboxs)

    return [bboxs, ids]

def augmentAruco(bbox, id, img, imgAug, drawId = True):

    tl = bbox[0][0][0], bbox[0][0][1]
    tr = bbox[0][1][0], bbox[0][1][1]
    br = bbox[0][2][0], bbox[0][2][1]
    bl = bbox[0][3][0], bbox[0][3][1]

    h,w,c = imgAug.shape

    pts1 = np.array([tl,tr,br,bl])
    pts2 = np.float32([[0,0],[w,0],[w,h],[0,h]])
    matrix, _ = cv2.findHomography(pts2,pts1)
    imgOut = cv2.warpPerspective(imgAug,matrix,(img.shape[1],img.shape[0]))
    cv2.fillConvexPoly(img,pts1.astype(int),(0,0,0))
    imgOut = img + imgOut

    if drawId:

        cv2.putText(imgOut, str(id), tl, cv2.FONT_HERSHEY_PLAIN,2,(255,0,255),2)

    return imgOut




def main():
    cap = cv2.VideoCapture(2)
    cap.set(3, 1280)
    cap.set(4, 720)
    cap.set(5, 120)
    imgAug = cv2.imread("/home/dong/PycharmProjects/testDemo/images/AugmentImg/9.jpg")
    augDics = loadAugImages("/home/dong/PycharmProjects/testDemo/images/AugmentImg/")

    while True:
        success, img = cap.read()
        arucoFound = findArucoMarker(img)

        #loop through all the markers and augment each one
        if len(arucoFound[0])!=0:
            for bbox,id in zip(arucoFound[0],arucoFound[1]):
                if int(id) in augDics:
                    img = augmentAruco(bbox,id,img,augDics[int(id)])

        cv2.imshow("Image",img)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()