
class area3D():
    def __init__(self,worldPos1,worldPos2):
        # self.worldPos1 = worldPos1
        # self.worldPos2 = worldPos2
        self.init_X = min(worldPos1[0], worldPos2[0])
        self.init_Y = min(worldPos1[1], worldPos2[1])
        self.init_Z = min(worldPos1[2], worldPos2[2])
        self.length = abs(worldPos1[0]-worldPos2[0])
        self.width = abs(worldPos1[1]-worldPos2[1])
        self.height = abs(worldPos1[2]-worldPos2[2])
        self.pt1  = worldPos1
        self.pt2 = worldPos2


