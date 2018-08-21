# -*- coding: utf-8 -*-
"""
Created on Tue Aug 14 14:45:49 2018

@author: kk
"""


import numpy as np
import math
import xlrd
from datetime import date,datetime

def read_excel(num):
    File = xlrd.open_workbook('contour.xlsx')
    print(File.sheet_names)
    sheet = File.sheet_by_index(num)
    x = sheet.col_values(0)
    y = sheet.col_values(1)
    for i in range(len(x)):
        print(1)
        contour[i] = [x[i],y[i]]
    return contour 

def Contour(contour):
    # 输入格式为 nx2的矩阵，每一行表示轮廓点的坐标（x,y）
    # 输出angle表示倾斜角，L是长边，W是短边
    clen = len(contour)
    x,y = np.zeros(clen), np.zeros(clen)
    for i in range(clen):
        x[i] = contour[i][0]
        y[i] = contour[i][1]
    
    xminind,xmaxind = np.argmin(x), np.argmax(x)
    yminind,ymaxind = np.argmin(y), np.argmax(y)
    
    point1x, point1y = x[xminind], y[xminind]
    point2x, point2y = x[yminind], y[yminind]
    point3x, point3y = x[xmaxind], y[xmaxind]
    point4x, point4y = x[ymaxind], y[ymaxind]
    
    angle12 = math.atan2(point2y-point1y, point2x-point1x)*180/math.pi
    angle23 = math.atan2(point3y-point2y, point3x-point2x)*180/math.pi
    
    if abs((angle12+90-angle23)%360) < 5: 
        Rotate = 0
       
    else:

        Rotate = 45/180*math.pi
        xnew = x*math.cos(Rotate) - y*math.sin(Rotate)
        ynew = x*math.sin(Rotate) + y*math.cos(Rotate)
        x = xnew
        y = ynew
        
        xminind,xmaxind = np.argmin(x), np.argmax(x)
        yminind,ymaxind = np.argmin(y), np.argmax(y)
        
        point1x, point1y = x[xminind], y[xminind]
        point2x, point2y = x[yminind], y[yminind]
        point3x, point3y = x[xmaxind], y[xmaxind]
        point4x, point4y = x[ymaxind], y[ymaxind]
        
        angle12 = math.atan2(point2y-point1y, point2x-point1x)*180/math.pi
        angle23 = math.atan2(point3y-point2y, point3x-point2x)*180/math.pi
        
    
    edge1 = math.sqrt( (point1x-point2x)**2 + (point1y-point2y)**2 )
    edge2 = math.sqrt( (point2x-point3x)**2 + (point2y-point3y)**2 )
    edge3 = math.sqrt( (point3x-point4x)**2 + (point3y-point4y)**2 )
    edge4 = math.sqrt( (point4x-point1x)**2 + (point4y-point1y)**2 )
    
    edge13 = (edge1+edge3)/2 
    edge24 = (edge2+edge4)/2 
    
    if edge13 > edge24:
        L = edge13
        W = edge24
        angle = angle23
    else:
        L = edge24
        W = edge13
        angle = angle12
        
    angle = angle-Rotate*180/math.pi
    print(angle)

    return angle, L, W

data = read_excel(1)
print(data)
#contour = [[1,2], [3,4],[3,0],[5,2], [2,3]]
angle,L,W = Contour(data)
