# -*- coding: utf-8 -*-
"""
Created on Fri Aug  3 14:26:57 2018

@author: kk
"""
import math
import numpy as np
from scipy import interpolate

def dist(Pos1=[0,0,0], Pos2=[0,0,0]):
    x1,y1,z1 = Pos1
    x2,y2,z2 = Pos2
    return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)

class pid:
    def __init__(self):
        self.K = [0.15, 0.03, 0]
        self.errorPre = [0,0,0]
        self.errorI = [0,0,0]
       
        
    def update(self, error=[0,0,0]):
        P,I,D = [0,0,0],[0,0,0],[0,0,0]
        Control = [0,0,0]
        for i in range(3):
            P[i] = error[i]*self.K[0]
            self.errorI[i] = self.errorI[i] + error[i]
            I[i] = self.errorI[i]*self.K[1]
            D[i] = (error[i] - self.errorPre[i])* self.K[2]
            Control[i] = P[i] + I[i] + D[i]
            Control[i] = (Control[i] < 20)*Control[i] + (Control[i]>=20)*20
        
        self.errorPre = error
        return Control

def RotateObj(deltaAngle, swift):
    swift.set_wrist(90,True)
    swift.set_pump(True)
    swift.set_wrist((90+deltaAngle)%180)
    swift.set_pump(False)
    swift.set_wrist(90)

class Action():
    def __init__(self):
        self.Position1 = [200,100,0]
        self.Position2 = [200,-100,0]
        self.Angle1 = 0
        self.Angle2 = 0
        self.Height = 100
        
    def move(self, Swift):
        x,y,z = Swift.get_position
        Swift.set_position(x,y,self.Height) # 初始位置抬升
        Swift.set_wrist(90)
        x,y,z = self.Position1
        Swift.set_position(x,y,self.Height) # 移动到起始位置正上方
        Swift.set_position(x,y,z-2) # 移动到物体上表面
        Swift.set_pump(True)
        Swift.set_position(x,y,self.Height) # 抬升物体
        x,y,z = self.Position2
        Swift.set_position(x,y,self.Height) # 移动到目标点正上方
        Swift.set_wrist((self.Angle2-self.Angle1+90)%180) # 转动到目标角度
        Swift.set_position(x,y,z-2) # 移动到指定位置
        Swift.set_pump(False)
        Swift.set_position(x,y,z+10) # 离开物体上表面
        
        
    
    
def StrightPath(Position1 = [200, 100, 0], Position2 = [200, -100, 0], Height = 120):
    Path = [[Position1[0],Position1[1],Height ]]
    Path.append([Position2[0],Position2[1],Height])
    Path.append(Position2)
    
    return Path

def BezierPath(Position1 = [150, 100, 0], Position2 = [250, -150, 0], ControlPoint=[[200,50,50],[200,-50,50]]):
    Path = [Position1]
    Path.append(ControlPoint[0])
    Path.append(ControlPoint[1])
    Path.append(Position2)
    l = len(Path)
    cumdis = [0,0,0,0]
    dis = [0,0,0]
    print(Path)
    for i in range(l-1):
        dis[i] = dist(Path[i],Path[i+1])
        cumdis[i+1] = sum(dis[0:i+1])
    
    
    t = np.zeros([l])
    for i in range(l):
        t[i] = np.array(cumdis[i]/cumdis[l-1])
    #print(t)
    
    step = 1/10
    tnew = np.arange(0,1+step,step)
    tlen = len(tnew)
    PathNew = np.zeros([tlen,3])
    
    for i in range(3):
        
        
        y = np.zeros(l)
        for j  in range(l):
            y[j] = np.array(Path[j][i])
        #print(y)    
        f = interpolate.interp1d(t,y)
        ynew = f(tnew)
        
        for  j in range(tlen):
            PathNew[j][i] = ynew[j]
    
    return PathNew
 

def GenControlPoint(obj1=None,obj2=None, obst=None):
    
    ControlPoint = [[0,0,0],[0,0,0]]
    if obst == None:
        ControlPoint[0] = [obj1.Center[0], obst.Center[1], obj1.H+10]
        ControlPoint[1] = [obj2.Center[0], obj2.Center[1], obj2.H+10]
        return ControlPoint, 0
        
    eps = 0.0001
    L0,W0,H0 = obst.L,obst.W,obst.H
    L1,W1,H1 = obj1.L,obj1.W,obj1.H
    Position0 = obst.Center[0],obst.Center[1],obst.H
    Position1 = obj1.Center[0],obj1.Center[1],obj1.H
    Position2 = obj2.Center[0],obj2.Center[1],obj2.H
    x0,y0,z0 = Position0
    x1,y1,z1 = Position1
    x2,y2,z2 = Position2
    

    if obst.H+obj1.H < 120:
    
        k1 = -(y1-y2)/(x1-x2+eps)
        b1 = -(y1+k1*x1)
        
        k2 = -1/k1    
        b2 = -(y1+k2*x1)    
        
        dist1 = abs(y2+k2*x2+b2)/math.sqrt(1+k2**2)
        dist2 = abs(y0+k2*x0+b2)/math.sqrt(1+k2**2)
        
        theta = (math.atan(-k1)-obst.Angle/180*math.pi)%(math.pi/2)
        if theta > math.atan(W0/L0) and theta < math.pi/2:
            dist3 = W0/2/math.sin(theta)
        elif theta >0:
            dist3 = L0/2/math.sin(math.pi/2-theta)
        else:
            print('ERROR')
        
        dist4 = (W0+W1)/W0 * dist3
        
        con1 = (dist2-dist4)/dist1
        con2 = (dist2+dist4)/dist1
        print([con1, con2])
        
        dx = x2-x1
        dy = y2-y1
        
        ControlPoint[0] = [x1+dx*con1, y1+dy*con1, H0+H1+10]
        ControlPoint[1] = [x1+dx*con2, y1+dy*con2, H0+H1+10]
        
        deltaAngle = obst.Angle-obj1.Angle
        
    elif x0>150:
        deltaAngle = 90+obj1.Angle
        diag = math.sqrt(L0**2+W0**2)
        ControlPoint[0] = [x0-W1/2-diag/2-5, y1, H1+10]
        ControlPoint[1] = [x0-W1/2-diag/2-5, y2, H1+10]
        
    else: 
        print("Hard Solve")
        
    
    
    return ControlPoint, deltaAngle

    
def CheckPath(Path):
    n = len(Path)
    RlimitDown,RlimitUp = 140,330
    ZlimitDown,ZlimitUp = -3,160
    for i in range(n):
        x = Path[i][0]
        y = Path[i][1]
        z = Path[i][2]
        r = math.sqrt(x**2+y**2)
        if r<RlimitDown or r>RlimitUp:
            print('Radius of Path is forbidden')
            return False
        elif z<ZlimitDown or z>ZlimitUp:
            print('Height of Path is forbidden')
            return False
        else: 
            return True
        

        