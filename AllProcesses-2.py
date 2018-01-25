# Multiprocessing Robot

import time
from BrickPi import *   #import BrickPi.py file to use BrickPi operations
from MultiMotorDriving import *
from MPU9250 import *
import RPi.GPIO as GPIO
from multiprocessing import *
from picamera.array import PiRGBArray
from picamera import*

import Navigation
import WorkingCV
import turretControl


if __name__ == '__main__':
    ctTurretPipe, ctCameraPipe = Pipe()
    ntTurretPipe, ntNavPipe = Pipe()
    cnCameraPipe, cnNavPipe = Pipe()
    controlPipe, firePipe = Pipe()
    
    nav = Process(target= Navigation.navigate, args= (cnNavPipe, ntNavPipe,))
    cam = Process(target= WorkingCV.sphereSearch, args= (ctCameraPipe, cnCameraPipe,))
    turret = Process(target= turretControl.turretCommand, args= (ctTurretPipe, ntTurretPipe, controlPipe,))
    fire = Process(target= turretControl.fireCommand, args= (firePipe,))

    
    nav.start()
    cam.start()
    turret.start()
    fire.start()
    
    nav.join()
    cam.join()
    turret.join()
    fire.join()

    
    
    
    
    
    
    
    
