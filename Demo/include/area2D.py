
class area2D():
    def __init__(self,point1,point2):
        self.init_X = min(point1[0], point2[0])
        self.init_Y = min(point1[1], point2[1])
        self.length = abs(point1[0]-point2[0])
        self.width = abs(point1[1]-point2[1])



