from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import numpy as np
import math
import time


plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.sans-serif'] = ['SimHei']


def plot_opaque_cube(x=10, y=20, z=30, dx=40, dy=50, dz=60):
    fig = plt.figure(num=1)
    ax = fig.add_subplot(1, 1, 1, projection='3d')


    xx = np.linspace(x, x+dx, 2)
    yy = np.linspace(y, y+dy, 2)
    zz = np.linspace(z, z+dz, 2)

    xx, yy = np.meshgrid(xx, yy)

    ax.plot_surface(xx, yy, z)
    ax.plot_surface(xx, yy, z+dz)

    yy, zz = np.meshgrid(yy, zz)
    ax.plot_surface(x, yy, zz)
    ax.plot_surface(x+dx, yy, zz)

    xx, zz = np.meshgrid(xx, zz)
    ax.plot_surface(xx, y, zz)
    ax.plot_surface(xx, y+dy, zz)
    # ax.set_xlim3d(-dx, dx*2, 20)
    # ax.set_xlim3d(-dx, dx*2, 20)
    # ax.set_xlim3d(-dx, dx*2, 20)
    #plt.title("Cube")
    plt.show()
    



def plot_linear_cube(x, y, z, dx, dy, dz, color='blue'):
    fig = plt.figure()
    ax = Axes3D(fig)
    xx = [x, x, x+dx, x+dx, x]
    yy = [y, y+dy, y+dy, y, y]
    kwargs = {'alpha': 1, 'color': color}
    ax.plot3D(xx, yy, [z]*5, **kwargs)
    ax.plot3D(xx, yy, [z+dz]*5, **kwargs)
    ax.plot3D([x, x], [y, y], [z, z+dz], **kwargs)
    ax.plot3D([x, x], [y+dy, y+dy], [z, z+dz], **kwargs)
    ax.plot3D([x+dx, x+dx], [y+dy, y+dy], [z, z+dz], **kwargs)
    ax.plot3D([x+dx, x+dx], [y, y], [z, z+dz], **kwargs)
    #plt.title('Cube')
    plt.show()
    
    
    

def ShowState(Base,Target=None,ShowAxes=True,xlim=[-500,500],ylim=[-500,500],Height=500,obstacle1=None, obstacle2 = None, obstacle3 = None, obstacle4 = None): 
    fig = plt.figure()
    ax=fig.add_subplot(111,projection="3d")
    plt.xlim(xlim)
    plt.ylim(ylim)
    ax.scatter(0,0,Height,color='gray')
    if Target is not None:
        ax.scatter(Target[0],Target[1],Target[2])
    Now=Base
    while(1):
        if ShowAxes==True:
            ax.plot((Now.Origin[0],Now.End[0]),(Now.Origin[1],Now.End[1]),(Now.Origin[2],Now.End[2]),color='black')
        Square1,Square2=Now.Box[:4],Now.Box[4:]
        x, y, z = [x[0] for x in Square1], [x[1] for x in Square1], [x[2] for x in Square1]
        for i in range(3):
            ax.plot((x[i], x[i + 1]), (y[i], y[i + 1]), (z[i], z[i + 1]),color='red')
        ax.plot((x[-1], x[0]), (y[-1], y[0]), (z[-1], z[0]),color='red')
        x, y, z = [x[0] for x in Square2], [x[1] for x in Square2], [x[2] for x in Square2]
        for i in range(3):
            ax.plot((x[i], x[i + 1]), (y[i], y[i + 1]), (z[i], z[i + 1]),color='red')
        ax.plot((x[-1], x[0]), (y[-1], y[0]), (z[-1], z[0]),color='red')
        for i in range(4):
            ax.plot((Square1[i][0],Square2[i][0]),(Square1[i][1],Square2[i][1]),(Square1[i][2],Square2[i][2]),color='red')
        if Now.Next is None:
            break
        else:
            Now=Now.Next
            
    for num in range(4):
        obstacle = eval('obstacle'+str(num+1)) 
        if obstacle == None:
            break
        elif obstacle.Type == 'cuboid':
            alpha = obstacle.Angle/180*math.pi
            L,W = obstacle.L, obstacle.W
            x0 = obstacle.Center[0]
            y0 = obstacle.Center[1]
            x1 = x0-L/2*math.cos(alpha) + W/2*math.sin(alpha)
            y1 = y0-W/2*math.cos(alpha) - L/2*math.sin(alpha)
            x2 = x1+L*math.cos(alpha)
            y2 = y1+L*math.sin(alpha)
            x3 = 2*x0-x1
            y3 = 2*y0-y1
            x4 = x1-W*math.sin(alpha)
            y4 = y1+W*math.cos(alpha)
            
            z0 = 0
    
            dz = obstacle.H
            color = obstacle.Color
            xx = [x1, x2, x3, x4, x1]
            yy = [y1, y2, y3, y4, y1]
            kwargs = {'alpha': 1, 'color': color}
            ax.plot3D(xx, yy, [z0]*5, **kwargs)
            ax.plot3D(xx, yy, [z0+dz]*5, **kwargs)
            ax.plot3D([x1, x1], [y1, y1], [z0, z0+dz], **kwargs)
            ax.plot3D([x2, x2], [y2, y2], [z0, z0+dz], **kwargs)
            ax.plot3D([x3, x3], [y3, y3], [z0, z0+dz], **kwargs)
            ax.plot3D([x4, x4], [y4, y4], [z0, z0+dz], **kwargs)
        elif obstacle.Type == 'cylinder':
            
            pass
    
            
            
    plt.show()
    #time.sleep(1)
    plt.close(fig) 
    
    return fig

def ShowOnePart(Part,Target=None,ShowAxes=True,xlim=[-500,500],ylim=[-500,500],Height=400):
    fig=plt.figure()
    ax=fig.add_subplot(111,projection="3d")
    plt.xlim(xlim)
    plt.ylim(ylim)
    ax.scatter(0,0,Height,color='gray')
    if Target is not None:
        ax.scatter(Target[0],Target[1],Target[2])
    Square1, Square2 = Part.Box[:4], Part.Box[4:]
    x, y, z = [x[0] for x in Square1], [x[1] for x in Square1], [x[2] for x in Square1]
    for i in range(3):
        ax.plot((x[i], x[i + 1]), (y[i], y[i + 1]), (z[i], z[i + 1]), color='red')
    ax.plot((x[-1], x[0]), (y[-1], y[0]), (z[-1], z[0]), color='red')
    x, y, z = [x[0] for x in Square2], [x[1] for x in Square2], [x[2] for x in Square2]
    for i in range(3):
        ax.plot((x[i], x[i + 1]), (y[i], y[i + 1]), (z[i], z[i + 1]), color='red')
    ax.plot((x[-1], x[0]), (y[-1], y[0]), (z[-1], z[0]), color='red')
    for i in range(4):
        ax.plot((Square1[i][0], Square2[i][0]), (Square1[i][1], Square2[i][1]), (Square1[i][2], Square2[i][2]),
                color='red')
    plt.show()

