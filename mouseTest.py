from Detector import *
from mouseControl import mouseControl
import tag
import pyautogui
import sys
import numpy as np
from  tag import Direction
import cv2
import cv2.aruco as aruco
import glob
import os
import time
import math
import matplotlib.pyplot as plt
from PIL import Image
import keyboard


def trasform(pt1 ,pt2,x,y):
    xlen = abs(pt1[0]-pt2[0])
    ylen = abs(pt1[1]-pt2[1])

    tras_X = abs(x-pt1[0])/xlen*1920
    tras_Y = abs(y-pt1[1])/ylen*1080
    return tras_X,tras_Y


# print(pyautogui.size())
def followmove():
    detector = Detector(debug=True, Blur=True, Sharpen=True)
    id_count = np.zeros(50)
    mouse = mouseControl()
    pt1 = [-0.12886278 ,-0.05650569]
    pt2 = [0.17258327, 0.13395831]
    while True:
        tag_list = detector.get_tag_list()
        for e in tag_list:
            id_count[e._id] += 1

        for tag in tag_list:
            if tag._id == 2:
                mouse.drawRecordPositon()
                # mouse.clearRecordPosition()
                sys.exit()
            print(tag.world_position,tag.Euler_angle,tag.direction)

            x,y = trasform(pt1,pt2,tag.world_position[0],tag.world_position[1])

            # print(x)
            mouse.mouseMove(x,y)
            mouse.setPosition()
            mouse.recordPosition()


def FakeMouse(clickDuration = 0.5):
    detector = Detector(debug=True, Blur=True, Sharpen=True)
    id_count = np.zeros(50)
    mouse = mouseControl()
    pt1 = [-0.18169751, -0.11869167]
    pt2 = [0.21221632, 0.16634992]
    leftTime1 =0
    leftTime2 =1
    rightTime1=0
    rightTime2=1
    Time1 = 0
    Time2 = 2
    while True:
        tag_list = detector.get_tag_list()
        for e in tag_list:
            id_count[e._id] += 1

        for tag in tag_list:
            # print(tag._id)
            # print(tag._id,tag.world_position, tag.Euler_angle, tag.direction)
            if tag._id==6:
                x, y = trasform(pt1, pt2, tag.world_position[0], tag.world_position[1])
                if tag.direction == Direction.front:

                    mouse.mouseMove(x, y)
                elif tag.direction == Direction.left:
                    leftTime2 = time.perf_counter()
                    if leftTime2 - leftTime1 > clickDuration:
                        mouse.leftMouseClick()
                        leftTime1 = time.perf_counter()

                elif tag.direction == Direction.right:
                    rightTime2 = time.perf_counter()
                    if rightTime2 - rightTime1 > clickDuration:
                        mouse.rightMouseClick()
                        rightTime1 = time.perf_counter()
            if tag._id ==4:
                Time2 = time.perf_counter()
                print(Time2)
                if Time2 - Time1 > 0.5:
                    if tag.direction == Direction.front:
                        pyautogui.keyDown('q')
                        pyautogui.keyUp('q')
                        print('q')
                    elif tag.direction == Direction.left:
                        pyautogui.keyDown('w')
                        pyautogui.keyUp('w')
                        print('w')

                    elif tag.direction == Direction.right:
                        pyautogui.keyDown('w')
                        pyautogui.keyUp('w')
                        print('e')

                    elif tag.direction == Direction.behind:
                        pyautogui.keyDown('r')
                        pyautogui.keyUp('r')
                        print('r')

                    else:
                        pass
                    Time1 = time.perf_counter()

        else:
                pass

def FakeKeyboard(clickDuration = 0.5):
    detector = Detector(debug=True, Blur=True, Sharpen=True)
    id_count = np.zeros(50)
    mouse = mouseControl()
    pt1 = [-0.18169751, -0.11869167]
    pt2 = [0.21221632, 0.16634992]
    leftTime1 =0
    leftTime2 =1
    Time11=0
    Time22=2
    Time1 = 0
    Time2 = 2
    while True:
        tag_list = detector.get_tag_list()
        for e in tag_list:
            id_count[e._id] += 1

        for tag in tag_list:
            # print(tag._id)
            # print(tag._id,tag.world_position, tag.Euler_angle, tag.direction)
            # if tag._id==6:
            #     Time22 = time.perf_counter()
            #
            #     if Time22 - Time11 > 0.5:
            #         if tag.direction == Direction.front:
            #             pyautogui.keyDown('a')
            #             time.sleep(0.3)
            #
            #             pyautogui.keyUp('a')
            #             print('a')
            #         elif tag.direction == Direction.left:
            #             pyautogui.keyDown('d')
            #             time.sleep(0.3)
            #
            #             pyautogui.keyUp('d')
            #             print('d')
            #
            #         elif tag.direction == Direction.right:
            #             pyautogui.keyDown('d')
            #             time.sleep(0.3)
            #
            #             pyautogui.keyUp('d')
            #             print('d')
            #
            #         elif tag.direction == Direction.behind:
            #             pyautogui.keyDown('a')
            #             time.sleep(0.3)
            #
            #             pyautogui.keyUp('a')
            #             print('a')
            #
            #         else:
            #             pass
            #         Time1 = time.perf_counter()

            if tag._id ==4:
                Time2 = time.perf_counter()
                if Time2 - Time1 > 0.5:
                    if tag.direction == Direction.front:
                        pyautogui.keyDown('w')
                        time.sleep(0.3)
                        pyautogui.keyUp('w')
                        print('w')
                    elif tag.direction == Direction.left:
                        pyautogui.keyDown('a')
                        time.sleep(0.3)

                        pyautogui.keyUp('a')
                        print('s')

                    elif tag.direction == Direction.right:
                        pyautogui.keyDown('d')
                        time.sleep(0.3)

                        pyautogui.keyUp('d')
                        print('s')

                    elif tag.direction == Direction.behind:
                        pyautogui.keyDown('s')
                        time.sleep(0.3)

                        pyautogui.keyUp('s')
                        print('s')

                    else:
                        pass
                    Time1 = time.perf_counter()

        else:
                pass

def keyboardMouse():

    # print(0)
    # keyboard.wait('a')
    # # 在按下a之前后面的语句都不会执行，下面同理
    # print(1)
    # keyboard.wait('b')
    # print(2)
    # keyboard.wait('c')
    # print(3)
    # keyboard.wait()
    mouse = mouseControl()
    while True:
        keyboard.wait('q')
        mouse.leftMouseClick()
        print(3)


def image_process():
    im=Image.open("F:/python/webDemo/myplot.png")
    out= im.transpose(Image.FLIP_LEFT_RIGHT)
    out.save("F:/python/webDemo/myplot2.png")

if __name__ == "__main__":
    # FakeKeyboard()
    # img = image_process()
    keyboardMouse()

