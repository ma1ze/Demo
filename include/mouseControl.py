import pyautogui
import pyautogui as pag
import matplotlib.pyplot as plt
import  os
import  time
import pyautogui as pag

class mouseControl:
    def __init__(self,duration=0.001):
        self.duration = duration
        self.x = 0
        self.y = 0
        self.xRecord = []
        self.yRecord = []

    def mouseMove(self,x,y):
        pyautogui.moveTo(x,y,duration=self.duration)

    def leftMouseClick(self):
        pyautogui.click()

    def rightMouseClick(self):
        pyautogui.click(button='right')

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def setX(self,x):
        self.x = x

    def setY(self,y):
        self.y = y

    def setPosition(self):
        self.x,self.y = pag.position()

    def recordPosition(self):
        self.xRecord.append(self.getX())
        self.yRecord.append(self.getY())

    def clearRecordPosition(self):
        self.xRecord.clear()
        self.yRecord.clear()

    def drawRecordPositon(self):
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('the mouse log')
        # plt.plot(self.xRecord,self.yRecord) #轨迹图
        plt.scatter(self.xRecord, self.yRecord)  # 散点图
        # plt.hist(self.xRecord,self.yRecord) #直方图
        plt.show()


