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
from Models import Itzamna #Import the 3D model of the robot
from math import pi

logging.basicConfig( level=logging.INFO, format='%(levelname)s: %(asctime)s - %(message)s ', handlers=[logging.FileHandler("execution_log.log"), logging.StreamHandler()])

env = swift.Swift()
env.launch(realtime=True)

# b= geometry.Cuboid([0.5,0.5,0.5], collision=True)

# env.add(b)

class Simulation():
    def __init__(self) -> None:
        #Create the robots
        self.Itz = Itzamna.Itzamna()
        self.UR3 = UR3E.UR3E()    
        #Add the robots to the environment
        self.Itz.add_to_env(env)
        self.UR3.add_to_env(env)
        
        #add the bookshelf to the environment
        bookCase_path='temp\\bookshelf.stl'
        if not os.path.exists(bookCase_path):
            logging.error(f"Mesh file not found: {bookCase_path}")
            raise FileNotFoundError(f"Mesh file not found: {bookCase_path}")
        
        self.bookCaseMesh=geometry.Mesh(filename=bookCase_path,pose=SE3(0.8,0.8,0.8),color=(0.4,0.04,0.04), collision=True)
        env.add(self.bookCaseMesh) 
        
        self.ControlPanel = GUI.GUI(env, self.UR3, self.Itz)
        logging.info("Setting up environment")

    def main(self):
        self.Itz.q = self.ControlPanel.Itz.q
        self.UR3.q = self.ControlPanel.UR3.q
        # b.T = self.Itz.fkine(self.Itz.q)
        print(self.CollisionCheck(self.Itz, self.bookCaseMesh))
        env.step()
        
        # env.hold()
        pass

    def CollisionCheck(self, robot, shape):
        d, _, _ = robot.closest_point(robot.q, shape)
        print(d)
        if d is not None and d <= 0:
            return True
        else:
            return False

    #-----------------------------------Main----------------------------#          
    def check_Stop_press():
        #This function will check for a key press
        #Connect to read pin off arduino
        
        
        pass

    def run(self):
        #This part do threads to check for a key press
        t1 = threading.Thread(target=self.check_Stop_press)
        
        t2=threading.Thread(target=self.main)
        
        t1.start()  
        t2.start()
        
        t1.join()
        t2.join()

if __name__ == "__main__":
    Sim = Simulation()
    env.set_camera_pose([1.3,-2.3,1.3], [1.3,0,1.3])
    
    while True:
        Sim.main()
        Sim.ControlPanel.Refresh()