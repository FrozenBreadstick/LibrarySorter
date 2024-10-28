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
from Models.Robots import Itzamna #Import the 3D model of the robot
from math import pi
from pathlib import Path

logging.basicConfig( level=logging.INFO, format='%(levelname)s: %(asctime)s - %(message)s ', handlers=[logging.FileHandler("execution_log.log"), logging.StreamHandler()])

env = swift.Swift()
env.launch(realtime=True)

b= geometry.Cuboid([0.1,0.1,0.1], collision=True)

env.add(b)

class Simulation():
    def __init__(self) -> None:
        
        AssetsPath = Path("Models/Assests")
        #Create the robots
        self.Itz = Itzamna.Itzamna()
        self.UR3 = UR3E.UR3E()    
        
        logging.info("Setting up environment")
        
        #Add the robots to the environment
        self.Itz.add_to_env(env)
        self.UR3.base = self.UR3.base*SE3(-1.3,1,-0.1)  #(X,Z,-Y)
        print(self.UR3.base)
    
        self.UR3.add_to_env(env)
       
        self.EnvironmentAssets = dict(Asset0 = "bookshelf.stl", color0 = (0.39, 0.26, 0.15), pose0=SE3(-0.3,1.7,0),
                                 Asset1 = "laserMeshLong.stl", color1=(0.4,0.04,0.04), pose1=SE3(-2.4,-0.4,0),
                                 Asset2 = "laserMeshLong.stl", color2=(0.4,0.04,0.04), pose2=SE3(-2.4,2.3,0),
                                 Asset3 = "laserMeshShort.stl", color3=(0.4,0.04,0.04), pose3=SE3(-2.4,-0.4,0)@SE3.Rz(pi/2),
                                 Asset4 = "laserMeshShort.stl", color4=(0.4,0.04,0.04), pose4=SE3(3.75,-0.4,0)@SE3.Rz(pi/2),
                                 Asset5 = "table.stl", color5=(0.4,0.4,1), pose5=SE3(-1.3,1,0))
        self.Assets = []
        for i in range(6):
            if f'color{i}' in self.EnvironmentAssets:
                self.Assets.append(geometry.Mesh(str(AssetsPath / self.EnvironmentAssets[f'Asset{i}']), 
                                                 color=self.EnvironmentAssets[f'color{i}'],
                                                 pose=self.EnvironmentAssets[f'pose{i}'], 
                                                 collision=True))
            else:
                self.Assets.append(geometry.Mesh(str(AssetsPath / self.EnvironmentAssets[f'Asset{i}']), 
                                                 pose=self.EnvironmentAssets[f'pose{i}'], 
                                                 collision=True))
        for i in range(len(self.Assets)):
            env.add(self.Assets[i])
            logging.info(str("Adding Asset " + self.EnvironmentAssets[f'Asset{i}']))        
        
        self.ControlPanel = GUI.GUI(env, self.UR3, self.Itz)
       

    def main(self):
        self.Itz.q = self.ControlPanel.Itz.q
        self.UR3.q = self.ControlPanel.UR3.q
        for i in self.Assets:
            print(self.CollisionCheck(self.Itz, i))
        env.step()
        


    def CollisionCheck(self, robot, shape):
        for l in robot.links_3d:
            d, _, _ = l.closest_point(shape)
            if d is not None and d <= 0:
                return True
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