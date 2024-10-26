#Imports
from ir_support import * #Industrial Robotics Specific
import swift #Rendering for the robots in browser
from spatialmath import * #matrix math
from spatialmath.base import *
import spatialgeometry as geometry
import roboticstoolbox as rtb #robotics toolbox used for various calculations
from roboticstoolbox import *
import numpy as np #Additional math operations
from itertools import combinations
import os
import time
import logging
import sys
import threading
#custom other files
from UR3E import UR3E
from GUI import GUI
from Models import Itzamna, libBot #Import the 3D model of the robot

env = swift.Swift()
env.launch(realtime=True)

UR3 = UR3E.UR3E()
Itz = Itzamna.Itzamna()

UR3.add_to_env(env)
Itz.add_to_env(env)

t = geometry.Cuboid([0.5,0.5,0.5], collision=True, pose=SE3(0.8,0.8,0.8))
b= geometry.Cuboid([0.5,0.5,0.5], collision=True)
# vertex, faces, face_normals = tc.get_data()

env.add(t)
env.add(b)

ControlPanel = GUI.GUI(env, UR3, Itz)

def main():
    Itz.q = ControlPanel.Itz.q
    UR3.q = ControlPanel.UR3.q
    b.T = Itz.fkine(Itz.q)
    print(test(b, t))
    env.step()
    pass

def test(robot, shape):
    d, _, _ = robot.closest_point(shape)
    if d is not None and d <= 0:
        return True
    else:
        return False

#-----------------------------------Main----------------------------#
def EnvironmentSetup(): 
     rLibot = DHRobot3D.LibraryBot()
    
def check_Stop_press():
    #This function will check for a key press
    #Connect to read pin off arduino
    pass

def run():
    #This part do threads to check for a key press
    t1 = threading.Thread(target=check_Stop_press)
    t2=threading.Thread(target=main)
    

if __name__ == "__main__":
    env.set_camera_pose([1.3,-2.3,1.3], [1.3,0,1.3])
    while True:
        run()
        main()
        ControlPanel.Refresh()