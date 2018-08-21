# -*- coding: utf-8 -*-
"""
Created on Thu Aug  2 18:54:24 2018

@author: kk
"""

from uarm.wrapper import SwiftAPI

class uArmSwift:
    def __init__(self):
        self.swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'}, cmd_pend_size=2, callback_thread_pool_size=1)
        if not self.swift.connected:
            print('lose connect')
    
        self.swift.waiting_ready()
        
        device_info = self.swift.get_device_info()
        print(device_info)
        firmware_version = device_info['firmware_version']
        if firmware_version and not firmware_version.startswith(('0.', '1.', '2.', '3.')):
            self.swift.set_speed_factor(0.00005)
        
        self.swift.set_mode(0)
        
        self.speed = 500000
        
        self.swift.set_wrist(angle=90)
        
        self.wristAngle = self.swift.get_servo_angle(0, timeout=10)
    
    def set_position(self, x=100, y=0, z=100, wait=False):
        self.swift.set_position(x, y, z, speed=self.speed, wait=wait)
        
    def set_polar(self, stretch, rotation, height, wait=False):
        self.swift.set_polar(stretch, rotation, height, speed=self.speed, wait=wait)
        
    def set_servo_angle(self, num, angle, wait=False):
        if num<0 and num>3:
            print("num is wrong")
        self.swift.set_servo_angle(num, angle, wait, speed=self.speed, wait=wait)
        
    def set_wrist(self, angle = 90, wait=False):  # 第四电机
        self.swift.set_wrist(angle,wait)
        
    def set_pump(self, on = False):
        self.swift.set_pump(on)
        
    def set_buzzer(self, freq = 1000, duration = 1, wait=False):
        self.swift.set_buzzer(freq, duration, wait)
        
    def get_position(self):
        return self.swift.get_position()
    
    def get_servo_angle(self,id=0):
        return self.swift.get_servo_angle(id, timeout=10)
        
    def is_moving(self):
        return self.swift.get_is_moving()
        
    def disconnect(self):
        self.swift.disconnect()