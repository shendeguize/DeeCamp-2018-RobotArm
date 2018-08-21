import math
from math import pi


def cos(theta):
    return math.cos(theta / 180 * pi)


def sin(theta):
    return math.sin(theta / 180 * pi)


def PointInNewCoor(PointFixed, Coordinate, Origin):
    PointNew = [0, 0, 0]
    for i in range(3):
        PointNew[i] = sum(list(map(lambda p, v: p * v[i], PointFixed, Coordinate)))
    return list(map(lambda p, o: p + o, PointNew, Origin))


def GenBoxFromFourPoints(FourPoints):
    Box = []
    Ps = {}
    Ps[0], Ps[1], Ps[2], Ps[4] = FourPoints
    Ps[3] = list(map(lambda p0, p1, p2: p2 + p0 - p1, Ps[0], Ps[1], Ps[2]))
    for i in range(5, 8):
        Ps[i] = list(map(lambda pi_4, p0, p4: pi_4 + p4 - p0, Ps[i - 4], Ps[0], Ps[4]))
    for i in range(8):
        Box.append(Ps[i])
    return Box


def UpdateCoordinate(Part):
    if Part.Type == "Base":
        return
    if Part.Type == "ArmLvl":
        Part.Origin = Part.Pre.End
        Now=Part
        while (1):
            Now=Now.Pre
            if Now.Type=="TurnTable":
                break
        Part.Coordinate=Now.Coordinate
    else:
        if Part.Axe == "z":
            Part.Origin = Part.Pre.End
            Part.Coordinate = Part.Pre.Coordinate
            i_Pre, j_Pre, k_Pre = Part.Coordinate
            k_Turned = k_Pre
            i_Turned = list(map(lambda i, j: i * cos(Part.TurnedAngle) + j * sin(Part.TurnedAngle), i_Pre, j_Pre))
            j_Turned = list(map(lambda i, j: -i * sin(Part.TurnedAngle) + j * cos(Part.TurnedAngle), i_Pre, j_Pre))
            Part.Coordinate = [i_Turned, j_Turned, k_Turned]
        elif Part.Axe == "y":
            Part.Origin = Part.Pre.End
            Part.Coordinate = Part.Pre.Coordinate
            i_Pre, j_Pre, k_Pre = Part.Coordinate
            j_Turned = j_Pre
            i_Turned = list(map(lambda i, k: -k * sin(Part.TurnedAngle) + i * cos(Part.TurnedAngle), i_Pre, k_Pre))
            k_Turned = list(map(lambda i, k: k * cos(Part.TurnedAngle) + i * sin(Part.TurnedAngle), i_Pre, k_Pre))
            Part.Coordinate = [i_Turned, j_Turned, k_Turned]
        return


def UpdateBox(Part):
    Part.BoxFixed = GenBoxFromFourPoints(Part.FourPoints)
    Part.Box = deepcopy(Part.BoxFixed)
    for i in range(8):
        Part.Box[i] = PointInNewCoor(Part.BoxFixed[i], Part.Coordinate, Part.Origin)
    return


def UpdatePart(Part):
    UpdateCoordinate(Part)
    Part.End = PointInNewCoor(Part.ConnectPoint, Part.Coordinate, Part.Origin)
    UpdateBox(Part)
    if Part.Next is not None:
        UpdatePart(Part.Next)
    else:
        return


from copy import deepcopy


class Base:
    def __init__(self, Name):
        self.Name = Name
        self.Type = "Base"

        self.Origin = [0, 0, 0]
        self.Coordinate = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # i,j,k vector of coordinate system
        
        self.Length, self.Width, self.Height = 134, 123, 70
        L,W,H = self.Length, self.Width, self.Height
        self.FourPoints = [[W/2,-L/2,H] , [-W/2,-L/2,H],[-W/2,L/2,H], [W/2,-L/2,0]] # I II III V Octant.These four points are under self-fixed coordinate.
        self.BoxFixed = GenBoxFromFourPoints(self.FourPoints)
        self.Box = deepcopy(self.BoxFixed)

        self.End = [0, 0, H]

        self.Next = None
        
    def update(self):
        L, W, H = self.Length, self.Width, self.Height
        self.FourPoints = [[W/2,-L/2,H] , [-W/2,-L/2,H],[-W/2,L/2,H], [W/2,-L/2,0]]  # I II III V Octant.These four points are under self-fixed coordinate.
        self.BoxFixed = GenBoxFromFourPoints(self.FourPoints)
        self.Box = deepcopy(self.BoxFixed)
        self.End = [0, 0, H]


class TurnTable:
    def __init__(self, Name):
        self.Name = Name
        self.Type = "TurnTable"

        self.Origin = [0, 0, 0]
        self.Coordinate = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        
        self.Length, self.Width, self.Height = 142, 72, 60
        L,W,H = self.Length, self.Width, self.Height
        self.FourPoints = [[W/2, -L/2, H], [W/2, L/2, H], [-W/2, L/2, H], [W/2, -L/2, 0]]
        self.BoxFixed = GenBoxFromFourPoints(self.FourPoints)
        self.Box = deepcopy(self.BoxFixed)

        self.ConnectPoint = [13.2, 0, 36.1]
        self.End = deepcopy(self.ConnectPoint)

        self.TurnedAngle = 0
        self.LimInterval = [-90, 90]
        self.Axe = "z"
        self.Speed=40

        self.Pre = None
        self.Next = None
        
        self.center = [0,0,0]
        
    def update(self):
        L,W,H = self.Length, self.Width, self.Height
        self.FourPoints = [[W/2, -L/2, H], [W/2, L/2, H], [-W/2, L/2, H], [W/2, -L/2, 0]]
        self.BoxFixed = GenBoxFromFourPoints(self.FourPoints)
        self.Box = deepcopy(self.BoxFixed)

        self.ConnectPoint = self.center
        self.End = deepcopy(self.ConnectPoint)

        
        
        
    def Turn2Angle(self, Angle):
        self.TurnedAngle = Angle
        if self.TurnedAngle > self.LimInterval[1]:
            self.TurnedAngle = self.LimInterval[1]
        elif self.TurnedAngle < self.LimInterval[0]:
            self.TurnedAngle = self.LimInterval[0]
        UpdatePart(self)

    def TurnDeltaAngle(self, DeltaAngle):
        self.Turn2Angle(self.TurnedAngle + DeltaAngle)

    def ConnectPre(self, Pre):
        self.Pre = Pre
        self.Pre.Next = self
        UpdatePart(self)


class Arm:
    def __init__(self, Name, FourPoints,ConnectPoint):
        self.Name = Name
        self.Type = "Arm"

        self.Origin = [0, 0, 0]
        self.Coordinate = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        self.FourPoints = FourPoints
        self.BoxFixed = GenBoxFromFourPoints(self.FourPoints)
        self.Box = deepcopy(self.BoxFixed)

        self.ConnectPoint = ConnectPoint
        self.End = deepcopy(self.ConnectPoint)

        self.TurnedAngle = 0
        self.LimInterval = [-90, 90]
        self.Axe = "y"
        self.Speed=40

        self.Pre = None
        self.Next = None

    def Turn2Angle(self, Angle):
        self.TurnedAngle = Angle
        if self.TurnedAngle > self.LimInterval[1]:
            self.TurnedAngle = self.LimInterval[1]
        elif self.TurnedAngle < self.LimInterval[0]:
            self.TurnedAngle = self.LimInterval[0]
        UpdatePart(self)

    def TurnDeltaAngle(self, DeltaAngle):
        self.Turn2Angle(self.TurnedAngle + DeltaAngle)

    def ConnectPre(self, Pre):
        self.Pre = Pre
        self.Pre.Next = self
        UpdatePart(self)

class ArmLvl:
    def __init__(self,Name,FourPoints,ConnectPoint):
        self.Name=Name
        self.Type="ArmLvl"

        self.Origin = [0, 0, 0]
        self.Coordinate = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        self.FourPoints = FourPoints
        self.BoxFixed = GenBoxFromFourPoints(self.FourPoints)
        self.Box = deepcopy(self.BoxFixed)

        self.ConnectPoint = ConnectPoint
        self.End = deepcopy(self.ConnectPoint)

        self.Pre = None
        self.Next = None

    def ConnectPre(self, Pre):
        self.Pre = Pre
        self.Pre.Next = self
        UpdatePart(self)


class Hand:
    def __init__(self, Name, FourPoints):
        self.Name = Name
        self.Type = "Hand"

        self.Origin = [0, 0, 0]
        self.Coordinate = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        self.FourPoints = FourPoints
        self.BoxFixed = GenBoxFromFourPoints(self.FourPoints)
        self.Box = deepcopy(self.BoxFixed)

        self.ConnectPoint = [0,0,0]
        self.End = deepcopy(self.ConnectPoint)

        self.TurnedAngle = 0
        self.LimInterval = [-90, 90]
        self.Axe = "z"

        self.Pre = None
        self.Next = None

    def Turn2Angle(self, Angle):
        self.TurnedAngle = Angle
        if self.TurnedAngle > self.LimInterval[1]:
            self.TurnedAngle = self.LimInterval[1]
        elif self.TurnedAngle < self.LimInterval[0]:
            self.TurnedAngle = self.LimInterval[0]
        UpdatePart(self)

    def TurnDeltaAngle(self, DeltaAngle):
        self.Turn2Angle(self.TurnedAngle + DeltaAngle)

    def ConnectPre(self, Pre):
        self.Pre = Pre
        self.Pre.Next = self
        UpdatePart(self)


class Obstacle:
    def __init__(self, Center=[0,0], Size=None, Type=None, Angle=0, Color='blue'):
        self.Type = Type
        self.Center = Center
        self.Color = Color
        if self.Type == 'cuboid':
            self.Angle = Angle
            self.L,self.W,self.H = Size
            self.R = 0
        elif self.Type == 'cylinder':
            self.R,self.H = Size
            self.L,self.W = 0,0
        else:
            print('Error')
                