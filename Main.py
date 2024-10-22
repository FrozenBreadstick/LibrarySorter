#Imports
from ir_support import DHRobot3D #Industrial Robotics Specific
import swift #Rendering for the robots in browser
from spatialmath import * #matrix math
from spatialmath.base import *
import spatialgeometry as geometry
import roboticstoolbox as rtb #robotics toolbox used for various calculations
from roboticstoolbox import *
import numpy as np #Additional math operations
import os
import time
import logging
import sys
import threading
from libBot_Assets import libBot #Import the 3D model of the robot

def EnvironmentSetup(): 
     rLibot = DHRobot3D.LibraryBot()

def mainCode():
    
  
    
    pass
    
    
def check_Stop_press():
    #This function will check for a key press
    #Connect to read pin off arduino
    pass

def run():
    #This part do threads to check for a key press
    t1 = threading.Thread(target=check_Stop_press)
    t2=threading.Thread(target=mainCode)
    


if __name__ == "__main__":
    run()