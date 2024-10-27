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
from pathlib import Path

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
        
        logging.info("Setting up environment")
         
        #Add the robots to the environment
        self.Itz.add_to_env(env)
        self.UR3.add_to_env(env)
        self.UR3.base = self.UR3.base*SE3(1,1,1)
        print("UR3 Pose: ", self.UR3.base)
    
        """ # Define the path to 'bookshelf.stl' using pathlib
        bookCase_path = Path('LibrarySorter') / 'temp' / 'bookshelf.stl'

        # Check if the file exists
        if not bookCase_path.exists():
            logging.error(f"Mesh file not found: {bookCase_path}")
            raise FileNotFoundError(f"Mesh file not found: {bookCase_path}") """

        # Add the bookcase to the environment
        self.bookCaseMesh=geometry.Mesh(filename='/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/temp/modBookShelf.stl',#str(bookCase_path) #Convert path to string
                                        pose=SE3(-0.3,1.7,0),
                                        color=(0.4,0.04,0.04), 
                                        collision=True)
        env.add(self.bookCaseMesh) 
        logging.info("Bookshelf added to environment")
        
        self.ControlPanel = GUI.GUI(env, self.UR3, self.Itz)
       

    def main(self):
        self.Itz.q = self.ControlPanel.Itz.q
        self.UR3.q = self.ControlPanel.UR3.q
        # b.T = self.Itz.fkine(self.Itz.q)
        print(self.CollisionCheck(self.Itz, self.bookCaseMesh))
        env.step()
        
        env.hold()


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