# -*- coding: utf-8 -*-
"""
Created on Thu Aug  2 11:41:13 2018

@author: kk
"""

import math
from math import pi

def cos(theta):
    return math.cos(theta)

def sin(theta):
    return math.sin(theta)

d1 = 106.1
a = [13.2, 142, 158.8]
deltaZ = -74 # 大致
deltaR = 56.65

def Angle2XYZ(Angle1=0, Angle2=0, Angle3=0 ):
    
    Angle1 = float(Angle1)/180*pi
    Angle2 = float(Angle2)/180*pi
    Angle3 = float(Angle3)/180*pi

    Z = d1 + a[1]*cos(Angle2) + a[2]*cos(Angle2+Angle3) + deltaZ
    R = a[0]+a[1]*sin(Angle2) + a[2]*sin(Angle2+Angle3) + deltaR
    X = R*cos(Angle1)
    Y = R*sin(Angle1)
    
    return X,Y,Z

def XYZ2Angle(X = 0, Y=0, Z=0):
    
    Angle1 = math.atan2(Y, X)
    R = X/cos(Angle1)
    p = Z - d1 - deltaZ
    q = R - a[0] - deltaR
    Angle3 = math.acos( (p**2+q**2-a[1]**2-a[2]**2) / (2*a[1]*a[2]) )
 
    Angle2Y = -(a[2]*sin(Angle3))*p + (a[1]+a[2]*cos(Angle3))*q
    Angle2X = (a[1]+a[2]*cos(Angle3))*p + (a[2]*sin(Angle3))*q
    Angle2 = math.atan2(Angle2Y, Angle2X)
    
    Angle1 = round(Angle1*180/pi)
    Angle2 = round(Angle2*180/pi)
    Angle3 = round(Angle3*180/pi)

    return Angle1, Angle2, Angle3
    

def Camera2Arm(Cx,Cy,Cz):
    theta = 0
    k = 1
    deltaX, deltaY, deltaZ = 0,0,0
    
    Cx, Cy, Cz = Cx*k, Cy*k, Cz
    Ax = Cx*math.cos(theta) - Cy*math.sin(theta) + deltaX
    Ay = Cx*math.sin(theta) + Cy*math.cos(theta) + deltaY
    Az = deltaZ - Cz

    return Ax,Ay,Az