from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt

def ShowState(Base,Target=None,ShowAxes=True,xlim=[-500,500],ylim=[-500,500],Height=500):
    fig=plt.figure()
    ax=fig.add_subplot(111,projection="3d")
    plt.xlim(xlim)
    plt.ylim(ylim)
    ax.scatter(0,0,Height,color='gray')
    if Target is not None:
        ax.scatter(Target[0],Target[1],Target[2])
    Now=Base
    while(1):
        if ShowAxes==True:
            ax.plot((Now.Origin[0],Now.End[0]),(Now.Origin[1],Now.End[1]),(Now.Origin[2],Now.End[2]),color='blue')
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
    plt.show()
    return
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

