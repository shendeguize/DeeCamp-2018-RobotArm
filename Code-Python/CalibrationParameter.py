# -*- coding: utf-8 -*-
"""
Created on Fri Aug  3 15:32:03 2018

@author: kk
"""

import os
import sys
sys.path.append(os.path.join('C:/Users/kk/Desktop/DeeCamp项目实践/uArm-Python-SDK-2.0/'))
import ArmSwift
import Kinematics
import numpy as np
import math

eps = 0.00001

'''
Swift = ArmSwift.uArmSwift()
initX, initY, initZ = Swift.get_position()
Swift.set_position(initX, initY, initZ)
X, Y, Z = Swift.get_position()
Angle1, Angle2, Angle3 = Swift.get_servo_angle()

Angle1  = Angle1

Xsimu, Ysimu, Zsimu = Kinematics.Angle2XYZ(Angle1-90, 90-Angle2, Angle2+Angle3)

deltaX = X - Xsimu
deltaY = Y - Ysimu
deltaZ = Z - Zsimu
print([deltaX, deltaY, deltaZ])
'''
PositionArm = [[0,0,0],[-1,1,0]]
PositionCamera = [[10,5,10],[15,2,10]]
Ax1, Ay1, Az1 = PositionArm[0]
Ax2, Ay2, Az2 = PositionArm[1]
Cx1, Cy1, Cz1 = PositionCamera[0]
Cx2, Cy2, Cz2 = PositionCamera[1]

k = math.sqrt( ((Ax1-Ax2)**2+(Ay1-Ay2)**2) / ((Cx1-Cx2)**2+(Cy1-Cy2)**2+eps) )
Cx1,Cy1,Cx2,Cy2 = Cx1*k,Cy1*k,Cx2*k,Cy2*k
theta = math.atan2(Ay2-Ay1, Ax2-Ax1) - math.atan2(Cy2-Cy1, Cx2-Cx1) 
deltaX = Ax1 - ( Cx1*math.cos(theta) - Cy1*math.sin(theta) )
deltaY = Ay1 - ( Cx1*math.sin(theta) + Cy1*math.cos(theta) )
deltaZ = Az1 - (-Cz1)

print([theta,k])
print([deltaX, deltaY, deltaZ])

Cx3, Cy3, Cz3 = 13*k, 8*k, 8
Ax3 = Cx3*math.cos(theta) - Cy3*math.sin(theta) + deltaX
Ay3 = Cx3*math.sin(theta) + Cy3*math.cos(theta) + deltaY
Az3 = deltaZ - Cz3
print([Ax3, Ay3, Az3])

# Swift.disconnect()