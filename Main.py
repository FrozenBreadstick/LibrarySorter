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
import platform #To check the OS

logging.basicConfig(filename='Execution_log.log', level=logging.INFO, format='%(levelname)s: %(asctime)s - %(message)s ', 
                    # handlers=[logging.FileHandler("execution_log.log"), logging.StreamHandler()],
                    filemode='w')

env = swift.Swift()
env.launch(realtime=True)

b= geometry.Cuboid([0.1,0.1,0.1], collision=True)

env.add(b)

class Simulation():
    def __init__(self) -> None:
        #-----------------------------------Setup----------------------------#
    
        #Create the robots
        self.Itz = Itzamna.Itzamna()
        self.UR3 = UR3E.UR3E()     
        
        #EStop Variables
        self.Stopped = False
        self.CheckEStop = False

        logging.info("Setting up environment")
            
        #Add the robots to the environment
        self.Itz.add_to_env(env)
        self.UR3.base = self.UR3.base*SE3(-1.3,1.01,-0.1)  #(X,Z,-Y)
        self.UR3.add_to_env(env)
    
        if platform.system() == 'Windows':
            AssetsPath = Path("Models/Assests")
        
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
        
        else:
            exact_path_bookshelf = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/Models/Assests/modBookShelf.stl'
            exact_path_laserMeshLong = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/Models/Assests/laserMeshLong.stl'
            exact_path_laserMeshShort = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/Models/Assests/laserMeshShort.stl'
            exact_path_tableMesh = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/Models/Assests/table.dae'
            

            
            #Add the first long laser               
            self.laserMeshLong = geometry.Mesh(filename=exact_path_laserMeshLong,
                                                pose=SE3(-2.4,-0.4,0),
                                                color=(0.4,0.04,0.04), 
                                                collision=True)
            env.add(self.laserMeshLong)
            
            #Add the second long laser
            self.laserMeshLong2 = geometry.Mesh(filename=exact_path_laserMeshLong,
                                                pose=SE3(-2.4,2.3,0),
                                                color=(0.4,0.04,0.04), 
                                                collision=True)
            env.add(self.laserMeshLong2)
            
            #Add the first short laser
            self.laserMeshShort = geometry.Mesh(filename=exact_path_laserMeshShort,
                                                pose=SE3(-2.4,-0.4,0)@SE3.Rz(pi/2),
                                                color=(0.4,0.04,0.04), 
                                                collision=True)
            env.add(self.laserMeshShort)
            
            #Add the second short laser
            self.laserMeshShort2 = geometry.Mesh(filename=exact_path_laserMeshShort,
                                                pose=SE3(3.75,-0.4,0)@SE3.Rz(pi/2),
                                                color=(0.4,0.04,0.04), 
                                                collision=True)
            env.add(self.laserMeshShort2)
            
            #Add the bookshelf
            self.bookshelf = geometry.Mesh(filename=exact_path_bookshelf,
                                                pose=SE3(-0.3,1.7,0),
                                                color=(0.39, 0.26, 0.15), 
                                                collision=True)
            env.add(self.bookshelf)
            
            #Add the table
            self.table = geometry.Mesh(filename=exact_path_tableMesh,
                                                pose=SE3(-1.3,1,0),
                                                color=(0.4,0.4,1), 
                                                collision=True)
            env.add(self.table)
            
            self.bookRef=self.add_book_to_env(env,[])  
            print(self.bookRef)
        
            logging.info("3d models added to the environment")
            
            
        self.ControlPanel = GUI.GUI(env, self.UR3, self.Itz)
        self.Sensor = geometry.Mesh(str(AssetsPath / 'hemisphere.stl'))
        # env.add(self.Sensor)

    def add_book_to_env(self, env,bookReference):
        '''
        Function to add the books to the environment
        
        '''
        book1_offset = 0.05
        book2_offset = 0.065
        book3_offset = 0.089
        #Add the paths here for books for windows
        if platform.system() == 'Windows':
            exact_path_book1 = Path("Models/Assests/SmallBook.stl")
            exact_path_book2 = Path("Models/Assests/MediumBook.stl")
            exact_path_book3 = Path("Models/Assests/LargeBook.stl")
        
        else:
         exact_path_book1 = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/Models/Assests/SmallBook.stl'
         exact_path_book2 = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/Models/Assests/MediumBook.stl'
         exact_path_book3 = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/LibrarySorter/Models/Assests/LargeBook.stl'
         
        
        bookPosition = [SE3(-0.85,2,0.91),
                        SE3(-0.85-book1_offset,2,0.91),
                        SE3(-0.85 -2*book1_offset,2,0.91),
                        SE3(-0.85 -3*book1_offset,2,0.91),
                        SE3(-0.85 -3*book1_offset-book2_offset,1.85,0.91),
                        SE3(-0.85 -3*book1_offset-2*book2_offset,1.85,0.91),
                        SE3(-0.85 -3*book1_offset-3*book2_offset,1.85,0.91),
                        SE3(-0.85 -3*book1_offset-4*book2_offset,1.85,0.91),
                        SE3(-0.85 -3*book1_offset-4*book2_offset-book3_offset,1.66,0.91),
                        SE3(-0.85 -3*book1_offset-4*book2_offset-2*book3_offset,1.66,0.91),
                        SE3(-0.85 -3*book1_offset-4*book2_offset-3*book3_offset,1.66,0.91),
                        SE3(-0.85 -3*book1_offset-4*book2_offset-4*book3_offset,1.66,0.91)
                        ]
        
        bookRotation = [pi/2,
                        pi/2,
                        pi/2,
                        pi/2,
                        pi/2,
                        pi/2,
                        pi/2,
                        pi/2,
                        pi/2,
                        pi/2,
                        pi/2,
                        pi/2]
        

        
        bookInnitPose=[]
        
        bookReference = []
        
        
            
        for i in range(4):   
            pose=bookPosition[i]@SE3.Ry(bookRotation[i]) #Calculate the pose of the book
            bookInnitPose.append(pose)
            
            bookMesh=geometry.Mesh(filename=exact_path_book1,
                                   pose=pose,
                                   color=(0.4,0.04,0.04), 
                                   collision=True)
            
            bookReference.append(bookMesh)
            env.add(bookMesh)
            
        for i in range(4):  
            i=i+4 
            pose=bookPosition[i]@SE3.Ry(bookRotation[i]) #Calculate the pose of the book
            bookInnitPose.append(pose)
            
            bookMesh=geometry.Mesh(filename=exact_path_book2,
                                   pose=pose,
                                   color=(0.4,0.04,0.04), 
                                   collision=True)
            
            bookReference.append(bookMesh)
            env.add(bookMesh)
            
        for i in range(4):  
            i=i+8
            pose=bookPosition[i]@SE3.Ry(bookRotation[i]) #Calculate the pose of the book
            bookInnitPose.append(pose)
            
            bookMesh=geometry.Mesh(filename=exact_path_book3,
                                   pose=pose,
                                   color=(0.4,0.04,0.04), 
                                   collision=True)
            
            bookReference.append(bookMesh)
            env.add(bookMesh)
        
        return bookReference
        

         
    def main(self):
        self.Itz.q = self.ControlPanel.Itz.q
        self.UR3.q = self.ControlPanel.UR3.q
        self.Sensor.T = self.Itz.fkine(self.Itz.q) @ SE3.Rx(-pi/2)
        for i in self.Assets:
            # print(self.CollisionCheck(self.UR3, i))
            pass
        
        
    def CollisionCheck(self, robot, shape):
        if type(robot) == Itzamna.Itzamna or type(robot) == UR3E.UR3E:
            for l in robot.links_3d:
                d, _, _ = l.closest_point(shape)
                if d is not None and d <= 0:
                    return True
        else:
            d, _, _ = robot.closest_point(shape)
            if d is not None and d <= 0:
                return True
        return False
    #-----------------------------------Main----------------------------#          
    def check_Stop_press(self):
        #This function will check for a key press
        #Connect to read pin off arduino
        if self.Itz.EStop == True or self.ControlPanel.Stopped == True:
            self.Stopped = True
        else:
            self.Stopped = False

        if self.Stopped == True:
            self.CheckEStop = True
            if self.Itz.EStop == True:
                input("Press Enter when the robot path is clear: ")
                self.Itz.EStop = False
        if self.CheckEStop == True and self.Stopped == False:
            input("Press Enter when it is safe to resume: ")
            self.CheckEStop = False
        

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
    env.set_camera_pose([0.5,-2.3,1.3], [0.5,0,1.3])
    
    Sim.run()
    while True:
        if Sim.Stopped == False and Sim.CheckEStop == False:
            Sim.main()
        Sim.check_Stop_press()
        Sim.ControlPanel.Refresh()
        env.step()