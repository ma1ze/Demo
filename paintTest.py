from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from  area3D import *
def linearPlot(pt1,pt2,color = 'red'):
    ax.plot((pt1[0], pt2[0]), (pt1[1], pt2[1]), (pt1[2], pt2[2]), color=color)

def cubePlot(pt1,pt2,color='red'):
    length = abs(pt2[0]-pt1[0])
    width = abs(pt2[1] - pt1[1])
    height = abs(pt2[2] - pt1[2])
    pt3 = [pt1[0]+length,pt1[1],pt1[2]]
    pt4 = [pt1[0]+length,pt1[1],pt1[2]+height]
    pt5 = [pt1[0],pt1[1],pt1[2]+height]
    pt6 = [pt1[0],pt1[1]+width,pt1[2]+height]
    pt7 = [pt1[0], pt1[1] + width, pt1[2]]
    pt8 = [pt1[0]+length, pt1[1] + width, pt1[2]]
    linearPlot(pt1,pt3,color=color)
    linearPlot(pt4,pt3,color=color)
    linearPlot(pt4,pt5,color=color)
    linearPlot(pt1,pt5,color=color)
    linearPlot(pt1,pt7,color=color)
    linearPlot(pt2,pt4,color=color)
    linearPlot(pt2,pt6,color=color)
    linearPlot(pt2,pt8,color=color)
    linearPlot(pt6,pt7,color=color)
    linearPlot(pt5,pt6,color=color)
    linearPlot(pt3,pt8,color=color)
    linearPlot(pt7,pt8,color=color)



def rectangePlot(pt1,pt2,color = 'red'):
    length = abs(pt2[0] - pt1[0])
    width = abs(pt2[1] - pt1[1])
    height = abs(pt2[2] - pt1[2])
    pt3 = [pt1[0]+length,pt1[1],pt1[2]+height]
    pt4 = [pt1[0],pt1[1]+width,pt1[2]]
    linearPlot(pt1,pt3,color=color)
    linearPlot(pt2, pt3, color=color)
    linearPlot(pt2, pt4, color=color)
    linearPlot(pt1, pt4, color=color)

if __name__ == "__main__":

    # pt1 =[0,0,0]
    # pt2 =[10,10,10]
    area1 = area3D([0,0,0],[100,100,100])
    area2 = area3D([20,20,20],[200,200,200])

    ax = plt.subplot(111, projection='3d')
    cubePlot(area1.pt1, area1.pt2)
    cubePlot(area2.pt1, area2.pt2)

    # rectangePlot(pt1,pt2)
    ax.set_zlabel('z')
    ax.set_ylabel('y')
    ax.set_xlabel('x')
    plt.show()

