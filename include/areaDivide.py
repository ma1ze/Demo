from area3D import area3D
from area2D import area2D

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def if_area2Doverlay(area2D1,area2D2):
    if area2D1.init_X > area2D2.init_X + area2D2.length or area2D1.init_X+area2D1.length<area2D2.init_X or area2D1.init_Y>area2D2.init_Y+area2D2.width or area2D1.init_Y+area2D1.width < area2D2.init_Y:
        return False
    else:
        return True

def if_area3Doverlay(area3D1,area3D2):

    if (area3D1.init_X>area3D2.init_X + area3D2.length or area3D1.init_X+area3D1.length<area3D2.init_X) or (area3D1.init_Y>area3D2.init_Y + area3D2.width or area3D1.init_Y+area3D1.width<area3D2.init_Y) or (area3D1.init_Z>area3D2.init_Z + area3D2.height or area3D1.init_Z+area3D1.height<area3D2.init_Z):
        return False
    else:
        return True

class areaDivide():
    def __init__(self):
        self.area2Ds =[]
        self.area3Ds =[]
        self.ax = plt.subplot(111, projection='3d')

    def setWorkingSpace(self,worldPos1,worldPos2):
        self.workingSpace = area3D(worldPos1,worldPos2)

    def conArea2D(self,Newarea2D):
        flag = False
        for area2D in self.area2Ds:
            if if_area2Doverlay(Newarea2D,area2D):
                flag = True

        return flag

    def conArea3D(self,Newarea3D):
        flag =False
        for area3D in self.area3Ds:
            if if_area3Doverlay(Newarea3D,area3D):
                flag = True

    def addArea2D(self,area2D):
        if len(self.area2Ds!=0):
            if self.conArea2D(area2D):
                print("area has overlay")

            else:
                self.area2Ds.append(area2D)
        else:
            self.area2Ds.append(area2D)
    def addArea3D(self,area3D):
        if len(self.area3Ds)!=0:
            if self.conArea3D(area3D):
                print("area3d has overlay")
            else:
                self.area3Ds.append(area3D)

        else:
            self.area3Ds.append(area3D)

    def drawArea(self):
        for area3D in self.area3Ds:
            self.cubePlot(area3D.pt1, area3D.pt2)

        self.ax.set_zlabel('z')
        self.ax.set_ylabel('y')
        self.ax.set_xlabel('x')
        plt.show()

    def linearPlot(self,pt1, pt2, color='red'):
        self.ax.plot((pt1[0], pt2[0]), (pt1[1], pt2[1]), (pt1[2], pt2[2]), color=color)

    def cubePlot(self,pt1,pt2,color='red'):
        length = abs(pt2[0]-pt1[0])
        width = abs(pt2[1] - pt1[1])
        height = abs(pt2[2] - pt1[2])
        pt3 = [pt1[0]+length,pt1[1],pt1[2]]
        pt4 = [pt1[0]+length,pt1[1],pt1[2]+height]
        pt5 = [pt1[0],pt1[1],pt1[2]+height]
        pt6 = [pt1[0],pt1[1]+width,pt1[2]+height]
        pt7 = [pt1[0], pt1[1] + width, pt1[2]]
        pt8 = [pt1[0]+length, pt1[1] + width, pt1[2]]
        self.linearPlot(pt1,pt3,color=color)
        self.linearPlot(pt4,pt3,color=color)
        self.linearPlot(pt4,pt5,color=color)
        self.linearPlot(pt1,pt5,color=color)
        self.linearPlot(pt1,pt7,color=color)
        self.linearPlot(pt2,pt4,color=color)
        self.linearPlot(pt2,pt6,color=color)
        self.linearPlot(pt2,pt8,color=color)
        self.linearPlot(pt6,pt7,color=color)
        self.linearPlot(pt5,pt6,color=color)
        self.linearPlot(pt3,pt8,color=color)
        self.linearPlot(pt7,pt8,color=color)



    def rectangePlot(self,pt1,pt2,color = 'red'):
        length = abs(pt2[0] - pt1[0])
        width = abs(pt2[1] - pt1[1])
        height = abs(pt2[2] - pt1[2])
        pt3 = [pt1[0]+length,pt1[1],pt1[2]+height]
        pt4 = [pt1[0],pt1[1]+width,pt1[2]]
        self.linearPlot(pt1,pt3,color=color)
        self.linearPlot(pt2, pt3, color=color)
        self.linearPlot(pt2, pt4, color=color)
        self.linearPlot(pt1, pt4, color=color)


if __name__ == "__main__":
    # area1 = area2D([0,0],[100,100])
    # area2 = area2D([150,150],[200,200])

    # print(if_area2Doverlay(area1,area2))

    area1 = area3D([0,0,0],[100,100,100])
    area2 = area3D([120,120,120],[200,200,200])
    ax = plt.subplot(111, projection='3d')
    divideArea =areaDivide()
    divideArea.addArea3D(area1)
    divideArea.addArea3D(area2)
    divideArea.drawArea()
    # print(if_area3Doverlay(area1,area2))