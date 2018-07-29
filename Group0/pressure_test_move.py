#!/usr/bin/env python3

import os
import sys
import time
import cv2 as cv
import numpy as np 
from numpy import *
import python_vision
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from uarm.wrapper import SwiftAPI

"""
pressure test: move
"""
height = 480
width = 640


def Camera_Calibration(data):
	
	# get solvePnP parameters		
	coordinates = data[:,[0,1,2]]
	cameracoordinates = data[:,[3,4]]
	intrinsics = python_vision.GetIntrinsicsParmameters()
	intrinsicsmatrix = np.array([[intrinsics[2], 0 ,intrinsics[4]],
									[0 ,intrinsics[3],intrinsics[5]],
									[0,0,1]])
	dist_coef = np.zeros(4)

	# get transformation matrix
	_ret, rvec, tvec = cv.solvePnP(coordinates, cameracoordinates, intrinsicsmatrix, dist_coef)
	rotation, jacobian= cv.Rodrigues(rvec, jacobian= 0)
	translation = tvec

	return rotation,translation

def Get_Coordinates(x,y,z,rotation,translation):
	"""convert camera coordinates to 3D camera coordinates, z represents  camera focal length"""
	rotation = mat(rotation)
	translation = mat(translation)
	try:
		if x != 0.0 and y != 0.0 and z != -1000:
			cordinates = mat([[x],[y],[z]]) 
			new_cordinates = rotation.I * (cordinates - translation)
			return new_cordinates
	except ValueError:
		print("Calibration points error!")


def Arm_Init():
    """Robotic arm initiation"""
    swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'}, cmd_pend_size=2, callback_thread_pool_size=1)

    swift.waiting_ready()

    device_info = swift.get_device_info()
    print(device_info)
    firmware_version = device_info['firmware_version']
    if firmware_version and not firmware_version.startswith(('0.', '1.', '2.', '3.')):
        swift.set_speed_factor(0.00005)

    swift.set_mode(0)

    return swift


def Get_Calibration_Points(swift):
    """Collect arm and camera 3D coordinates"""

    numofpoints = 5
    data = np.zeros((numofpoints,6))
    for i in range(numofpoints):

    	if i == 0:
    		print("Start Calibration Process. Total points: %d" %numofpoints)
    	print("Please move the object to the point %d and hit enter to record" % (i+1))
    	getinput = input()
    	
    	cameracoordinates = 1000*python_vision.GetPoints()
    	print("The camera coordinates are: ",cameracoordinates)
    	print("Please move the arm to the object and hit enter")
    	getinput = input()
    	coordinates = swift.get_position()
    	print("The arm coordinates are: ",coordinates)
    	print("\r")

    	data[i,[0,1,2]] = coordinates
    	data[i,[3,4,5]] = cameracoordinates[0],cameracoordinates[1],cameracoordinates[2]/10

    return data

def CameraToArm_Affine_Transformation(data):
    """Get affine transformation matrix"""
    Arm_Coordinates = data[:,[0,1,2]]
    Camera_Coordinates = data[:,[3,4,5]]
    retval, affine, inlier = cv.estimateAffine3D(Arm_Coordinates,Camera_Coordinates)
    try:
    	if retval == 1:
    		rotation = affine[:,[0,1,2]]
    		translation = affine[:,[3]]
    		return rotation,translation

    except ValueError:
    	print("Calibration Points not correct. Please try again")
    	pass


def CameraToArm_Coordinates(x,y,z,rotation,translation):
    """convert camera 3D coordinates to arm 3D coordinates"""
    try:
    	if x != 0.0 and y != 0.0 and z != -1000:
    		x *= 1000
    		y *= 1000
    		z *= 100
    		cordinates = mat([[x],[y],[z]])
    		rotation = mat(rotation)
    		translation = mat(translation)
    		new_cordinates = rotation.I * (cordinates - translation)
    		return new_cordinates
    	else:
    		print("No object detected!")  
    		new_cordinates = mat([0,0,0])			
    		return new_cordinates
    		# pass
    except:
    	ValueError
    	pass





def main():


	# Robotic Arm Initiation
	swift = Arm_Init()
	speed = 100000
	# swift.reset(speed=speed)

	# Camera Initiation
	python_vision.init()

	# Calibration process
	data = Get_Calibration_Points(swift)
	# print(data)
	rotation,translation = CameraToArm_Affine_Transformation(data)
	while swift.connected:
		start = input()
		time.sleep( 5 )
		pos = python_vision.GetPoints()
		posGreen = python_vision.GetGreenPoints()
		new_coordinates = CameraToArm_Coordinates(pos[0],pos[1],pos[2],rotation,translation)
		new_GreenCoordinates = CameraToArm_Coordinates(posGreen[0],posGreen[1],posGreen[2],rotation,translation)

		swift.set_position(x= new_coordinates.item(0), y=new_coordinates.item(1), z=new_coordinates.item(2), speed=speed)
		time.sleep( 5 )
		swift.set_pump(on=True)
		time.sleep( 5 )
		swift.set_position(x= 144, y=-190, z=118, speed=speed)
		time.sleep( 2 )
		swift.set_position(x= 36, y=-180, z=52, speed=speed)
		time.sleep( 5 )
		swift.set_pump(on=False)
		time.sleep( 5 )
		swift.set_position(x= 144, y=-190, z=118, speed=speed)
		time.sleep( 5 )
		swift.set_position(x= new_GreenCoordinates.item(0), y=new_GreenCoordinates.item(1), z=new_GreenCoordinates.item(2), speed=speed)
		time.sleep( 5 )
		swift.set_pump(on=True)
		time.sleep( 5 )
		swift.set_position(x= 144, y=-190, z=118, speed=speed)
		time.sleep( 2 )
		swift.set_position(x= 36, y=-317, z=52, speed=speed)
		time.sleep( 5 )
		swift.set_pump(on=False)




    # while swift.connected:

    #     pos = python_vision.GetPoints()
    #     new_coordinates = CameraToArm_Coordinates(pos[0],pos[1],pos[2],rotation,translation)
    #     print(new_coordinates)

    #     swift.set_position(x= new_coordinates.item(0), y=new_coordinates.item(1), z=new_coordinates.item(2), speed=speed)


        
        # swift.set_position(x=150, y=100, z=150, speed=speed)
        # print(swift.get_position())    
        # swift.set_position(x=0, y=0, z=0, speed=speed)
        # swift.set_position(x=0, y=0, z=150, speed=speed)
        # swift.set_position(x=0, y=0, z=0, speed=speed)
        # swift.set_position(x=300, y=0, z=150, speed=speed)
        # swift.set_position(z=0)
        # swift.set_position(z=150)
        # swift.set_position(x=200, y=100, z=100)
        # swift.set_position(z=50)


        # swift.set_polar(stretch=200, rotation=90, height=150, speed=speed)
        # swift.set_polar(stretch=200, rotation=45, height=150, speed=speed)
        # swift.set_polar(stretch=200, rotation=135, height=150, speed=speed)
        # swift.set_polar(stretch=200, rotation=135, height=90, speed=speed)
        # swift.set_polar(stretch=200, rotation=135, height=200, speed=speed)
        # swift.set_polar(stretch=200, rotation=135, height=150, speed=speed)


        # swift.set_servo_angle(0, 45, speed=speed)
        # swift.set_servo_angle(0, 135, speed=speed)
        # swift.set_servo_angle(1, 45, speed=speed)
        # swift.set_servo_angle(1, 90)
        # swift.set_servo_angle(2, 45)


if __name__ == "__main__":
    main()