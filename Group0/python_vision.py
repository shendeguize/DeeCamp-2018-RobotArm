import numpy as np
import sys
import random

#try:
#    import thread
#except ImportError:
#    import _thread as thread

import threading
import os
import ctypes
from ctypes import *
dir_path = os.path.dirname(os.path.realpath(__file__))
lib = cdll.LoadLibrary(dir_path + '/build/librealsensedetection.so')
t1 = None
from numpy.ctypeslib import ndpointer

class CameraThread (threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
	def run(self):
		lib.Run();

def init():
	thread1 = CameraThread(1, "camera_thread")
	thread1.start()

def GetPoints():
	"""Get object coordinates X, Y, Z """
	lib.GetCentralPoint.restype = ctypes.POINTER(ctypes.c_float * 3)

	pos = np.zeros((3), dtype=np.float)

	pospointer = lib.GetCentralPoint().contents

	for i in range(0, 3):
		pos[i] = pospointer[i]

	return pos


def GetGreenPoints():
	"""Get object coordinates X, Y, Z """
	lib.GetGreenCentralPoint.restype = ctypes.POINTER(ctypes.c_float * 3)

	pos = np.zeros((3), dtype=np.float)

	pospointer = lib.GetGreenCentralPoint().contents

	for i in range(0, 3):
		pos[i] = pospointer[i]

	return pos



def GetIntrinsicsParmameters():
	""" Get camera intrinsics parameters: width, height, fx, fy, centralpoint X, centralpoint Y"""
	lib.GetIntrinsics.restype = ctypes.POINTER(ctypes.c_float * 6)
	Intrinsics = np.zeros((6), dtype = np.float)
	Intrinsicspointer = lib.GetIntrinsics().contents

	for i in range(0,6):
		Intrinsics[i] = Intrinsicspointer[i]

	return Intrinsics 




