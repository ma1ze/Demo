import os
import sys
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore,QtGui,QtWidgets
from PyQt5.QtWidgets import QApplication,QLabel,QWidget,QVBoxLayout
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QDialog, QLineEdit, QLabel
import include
from include.data_delete import del_files
from include.singlePicture_capture import CalibImage_collecting
from include.checkBoard_calib import calibrate

import sys
import cv2.aruco as aruco
import numpy as np
import cv2

loc = '/home/dong/PycharmProjects/testDemo/images/calib/'

# 相机参数 1080, 760, 120, 128, 0, 64, 0, 312
Width = 1080
Height= 760
Fps= 120
Brightness = 128
Contrast = 0
Saturation = 64
Hue = 0
Exposure = 312


# 创建一个类
class Interface(QWidget):
    def __init__(self):
        super().__init__()
        global Width, Height, Fps, Brightness, Contrast, Saturation, Hue, Exposure
        self.Width = Width
        self.Height = Height
        self.Fps = Fps
        self.Brightness = Brightness
        self.Contrast = Contrast
        self.Saturation = Saturation
        self.Hue = Hue
        self.Exposure = Exposure
        self.init_Menu()

    def init_Menu(self):

        self.resize(400,400)
        self.setFixedSize(400,400)
        self.move(800,80)
        # 这个方法调用我们下面写的,实现对话框居中的方法
        self.setWindowTitle('控制窗口')
        self.init_StartPage()

        self.show()
        self.Picture_Capture_Push.clicked.connect(self.on_pictuerCapture_clicked)
        self.Data_delete_Push.clicked.connect(self.on_dataReset_clicked)
        self.Camera_Calibration_Push.clicked.connect(self.on_cameraCalib_clicked)
        self.Camera_Set_Push.clicked.connect(self.on_cameraSet_clicked)


    def init_label(self):
        self.label0 = QLabel(self)
        self.label1 = QLabel(self)
        self.label2 = QLabel(self)
        self.label3 = QLabel(self)
        self.label4 = QLabel(self)
        self.label5 = QLabel(self)
        self.label0.setText("Camera: ")
        self.label1.setText("Width: "+ str(self.Width))
        self.label2.setText("Height: " + str(self.Height))
        self.label3.setText("Fps: "+ str(self.Fps))
        self.label4.setText("Exposure: "+str(self.Exposure))

        self.label0.setGeometry(20, 180, 100, 190)
        self.label1.setGeometry(20, 200, 100, 190)
        self.label2.setGeometry(20, 220, 100, 190)
        self.label3.setGeometry(20, 240, 100, 190)
        self.label4.setGeometry(20, 260, 100, 190)
        self.label5.setGeometry(20, 280, 100, 190)

    def rapaint_label(self):
        self.label1.setText("Width: "+ str(self.Width))
        self.label2.setText("Height: " + str(self.Height))
        self.label3.setText("Fps: "+ str(self.Fps))
        self.label4.setText("Exposure: "+str(self.Exposure))

    def init_StartPage(self):

        self.startFrame = QFrame(self)
        self.init_label()
        self.startVerticalLayout = QVBoxLayout(self.startFrame)
        self.Camera_Set_Push = QPushButton("相机参数设定")
        self.Picture_Capture_Push = QPushButton("采集标定照片")
        self.Camera_Calibration_Push = QPushButton("标定相机")
        self.Data_Collect_Push = QPushButton("测试数据采集")
        self.Data_delete_Push = QPushButton("重置")
        self.startVerticalLayout.addWidget(self.Camera_Set_Push)
        self.startVerticalLayout.addWidget(self.Picture_Capture_Push)
        self.startVerticalLayout.addWidget(self.Camera_Calibration_Push)
        self.startVerticalLayout.addWidget(self.Data_Collect_Push)
        self.startVerticalLayout.addWidget(self.Data_delete_Push)




    def on_cameraSet_clicked(self):
        width, ok1 = QInputDialog.getInt(self,"Width",'Width:',self.Width)
        height, ok2 = QInputDialog.getInt(self, "Height", 'Height:', self.Height)
        fps, ok3 = QInputDialog.getInt(self,"FPS","FPS:",self.Fps)
        exposure, ok4 =QInputDialog.getInt(self,"Exposure","Exposure",self.Exposure)
        self.Width = width
        self.Height = height
        self.Fps  = fps
        self.Exposure =exposure

        self.rapaint_label()


    def on_dataReset_clicked(self):
        del_files(loc)

    def on_pictuerCapture_clicked(self):
        CalibImage_collecting(loc, self.Width, self.Height, self.Fps, self.Brightness, self.Contrast, self.Saturation, self.Hue, self.Exposure)

    def on_cameraCalib_clicked(self):
        calibrate(loc)



app = QApplication(sys.argv)
demo = Interface()
sys.exit(app.exec_())
