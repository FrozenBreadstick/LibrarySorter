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
from math import pi

env = swift.Swift()
env.launch(realtime=True)



t = geometry.Cuboid([0.5,0.5,0.5], collision=True, pose=SE3(0.8,0.8,0.8))
b= geometry.Cuboid([0.5,0.5,0.5], collision=True)
# vertex, faces, face_normals = tc.get_data()

env.add(t)
env.add(b)

#ControlPanel = GUI.GUI(env, UR3, Itz)

def main():
    EnvironmentSetup(env)
    
    """ b.T = Itz.fkine(Itz.q)
    print(test(b, t)) ## not working
     """
    env.step()
    
   # env.hold()
    pass

def test(robot, shape):
    d, _, _ = robot.closest_point(shape)
    if d is not None and d <= 0:
        return True
    else:
        return False

#-----------------------------------Main----------------------------#
def EnvironmentSetup(env): 
   
    
    
    #Create the robots
    Itz = Itzamna.Itzamna()
    UR3 = UR3E.UR3E()    

    #Add the robots to the environment
    Itz.add_to_env(env)
    UR3.add_to_env(env)
    
    #add the bookshelf to the environment
    bookCase_path='LibrarySorter/temp/bookcase.dae'
    if not os.path.exists(bookCase_path):
        raise FileNotFoundError(f"Mesh file not found: {bookCase_path}")
    
    bookCaseMesh=geometry.Mesh(filename=bookCase_path,pose=SE3(0.8,0.8,0.8),color=(0.4,0.04,0.04))
    env.add(bookCaseMesh) 
    
    print("Setting up environment")
    
    
    
def check_Stop_press():
    #This function will check for a key press
    #Connect to read pin off arduino
    
    
    pass

def run():
    #This part do threads to check for a key press
    t1 = threading.Thread(target=check_Stop_press)
    
    t2=threading.Thread(target=main)
    
    t1.start()  
    t2.start()
    
    t1.join()
    t2.join()

def run2():
    main()

if __name__ == "__main__":
    env.set_camera_pose([1.3,-2.3,1.3], [1.3,0,1.3])
    run2()
    
    """ while True:
        ControlPanel.Refresh() """