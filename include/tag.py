import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
from angle_change import rotationVectorToEulerAngles

from enum import Enum

class Direction(Enum):
    front= 1
    left = 2
    right = 3
    behind = 4
    notSure = 0


def inRange(value,target,vari):
    if abs(value-target)<=vari:
        return True
    else:
        return False

class Tag(object):

    def __init__(self,_id,tvec,rvec):
        # The unique ID for the block
        self._id = _id
        self.rvec = rvec
        self.tvec =  tvec
        self.time = time.perf_counter()



    def setTvec(self, tvec):
        self.tvec = tvec

    def setRvec(self, rvec):
        self.rvec = rvec

    @property
    def world_position(self):
        # tvec = self.tvec *1000
        return self.tvec

    @property
    def Euler_angle(self):
        Euler_angle_ = rotationVectorToEulerAngles(self.rvec)
        Euler_angle = [Euler_angle_[0],Euler_angle_[1],Euler_angle_[2]]

        return Euler_angle

    @property
    def direction(self):
        if (inRange(self.Euler_angle[0],180,20) and inRange(self.Euler_angle[1],0,20) and inRange(self.Euler_angle[2],0,20)) or (inRange(self.Euler_angle[0],-180,20) and inRange(self.Euler_angle[1],0,20) and inRange(self.Euler_angle[2],0,20)):
            return Direction.front
        elif (inRange(self.Euler_angle[0],-180,20) and inRange(self.Euler_angle[1],0,20) and inRange(self.Euler_angle[2],-90,20)) or(inRange(self.Euler_angle[0],180,20) and inRange(self.Euler_angle[1],0,20) and inRange(self.Euler_angle[2],-90,20)):
            return Direction.left
        elif (inRange(self.Euler_angle[0],180,20) and inRange(self.Euler_angle[1],0,20) and inRange(self.Euler_angle[2],90,20)) or(inRange(self.Euler_angle[0],-180,20) and inRange(self.Euler_angle[1],0,20) and inRange(self.Euler_angle[2],90,20)):
            return Direction.right
        elif (inRange(self.Euler_angle[0],180,20) and inRange(self.Euler_angle[1],0,20) and inRange(self.Euler_angle[2],180,20)) or (inRange(self.Euler_angle[0],-180,20) and inRange(self.Euler_angle[1],0,20) and inRange(self.Euler_angle[2],180,20)):
            return Direction.behind
        else:
            return Direction.notSure

