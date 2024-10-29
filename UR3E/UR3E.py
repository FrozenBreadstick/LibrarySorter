##  @file
#   @brief UR3ER Robot defined by standard DH parameters with 3D model
#   @author Quoc Binh NGUYWN
#   @date Sep 17, 2024

import swift
import roboticstoolbox as rtb
from roboticstoolbox import DHLink, DHRobot, jtraj
import spatialmath.base as spb
from spatialmath import SE3
import spatialgeometry as geometry
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
import keyboard
import numpy as np
import matplotlib.pyplot as plt
import threading
import random
# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class UR3E(DHRobot3D):    
    def __init__(self):     
    
        # DH links
        links = self._create_DH()     
        
        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'base_rail', color0 = (0.5,0,0,1), #(0.2,0.2,0.2,1),      # color option only takes effect for stl file
                            link1 = 'slider_stl', color1 = (0.1, 0.2, 0.8),
                            link2 = 'shoulder_ur3_stl', color2 = (0.1, 0.2, 0.8),
                            link3 = 'upperarm_ur3_stl',color3 = (0.1, 0.2, 0.8),
                            link4 = 'forearm_ur3_stl',color4 = (0.1, 0.2, 0.8),
                            link5 = 'wrist1_ur3_stl',color5 = (0.1, 0.2, 0.8),
                            link6 = 'wrist2_ur3_stl', color6 = (0.1, 0.2, 0.8),
                            link7 = 'wrist3_ur3_stl', color7 = (0.1, 0.2, 0.8))
        
     
        # A joint config and the 3D object transforms to match that config
        qtest = [0,0,-pi/2,0,0,0,0]

        qtest_transforms = [
                            spb.transl(0,0,0), 
                            spb.transl(0,0,0) @ spb.trotx(-pi/2),                                  
                            spb.transl(0,0.242401,0) @ spb.rpy2tr(0,pi,pi/2, order = 'xyz'), 
                            spb.transl(0,0.242401,0.12002) @ spb.rpy2tr(0,pi,pi/2, order = 'xyz'),
                            spb.transl(0,0.48582,0.027114) @ spb.rpy2tr(0,pi,pi/2, order = 'xyz'), 
                            spb.transl(0,0.69901,0.027313) @ spb.rpy2tr(-pi/2,pi/2,pi, order = 'xyz'),
                            spb.transl(0.000385,0.69902,0.10833) @ spb.rpy2tr(-pi,pi/2,-pi/2, order= 'xyz'),
                            spb.transl(0.083763,0.697501,0.10932) @ spb.rpy2tr(-pi,0,-pi/2, order= 'xyz'),
                             ]
        

        self.EStop = False
        #The current path of this script
        current_path = os.path.abspath(os.path.dirname(__file__))

        super().__init__(links, link3D_names, name = 'UR3E', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.base = SE3(0,0,0) *SE3.Rx(pi/2)
        
        #Very Important for making sure the orientation of the DH visualiser to math the 3D model.
        self.q=qtest
        
    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        # d translation along z
        # a translation along x
        #alpha rotation about x
        #qlim Represents the joint limits in radians.

        links = [rtb.PrismaticDH(theta= pi, a= 0, alpha= pi/2, qlim= [-0.8, 0])]    # Prismatic Link
        a = [0,0.24365,0.21325, 0, 0, 0]
        d = [0.24189, 0, 0, 0.11235, 0.08535, -0.0819]
        alpha = [-pi/2, 0, 0, pi/2, pi/2, 0]
        qlim = [[-2*pi, 2*pi] for _ in range(6)]

        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
            links.append(link)
        return links

    def _DHVisualiser(self):
        """
        Visualise the DH model of the robot
        """
        lo= DHLink(theta=pi, a= 0, alpha= pi/2, qlim= [-0.8,0],sigma=1)
        l1 = DHLink(d=0.24189, a= 0, alpha= -pi/2, qlim= [-2*pi,2*pi])
        l2 = DHLink(d= 0, a= 0.24365, alpha= 0, qlim= [-2*pi,2*pi])
        l3 = DHLink(d= 0, a= 0.21325, alpha= 0, qlim= [-2*pi,2*pi])
        l4 = DHLink(d= 0.11235, a= 0, alpha= pi/2, qlim= [-2*pi,2*pi])
        l5 = DHLink(d= 0.08535, a= 0, alpha= pi/2, qlim= [-2*pi,2*pi])
        l6 = DHLink(d= -0.0819, a= 0, alpha= 0, qlim= [-2*pi,2*pi])

        robot = DHRobot([lo,l1, l2, l3,l4,l5,l6], name= 'my_robot')                             # Generate the model

        workspace = [-4, 4, -4, 4, -4, 4]                                           # Set the size of the workspace
        robot.q= [0.5,0,-pi/2,0,0,0,0]
        fig = robot.teach(robot.q, block= False, limits= workspace)                 # Open a menu to move the robot manually
        fig.hold()
    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        self.q = self._qtest        
        self.add_to_env(env)


        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        q_goal[0]=-0.8  #move rail link
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        
        for q in qtraj:
            self.q = q
            env.step(0.02)
    

        self.q=q_goal #last position 
        q_goal=self._qtest
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        for q in qtraj:
            self.q = q
            env.step(0.02)

        env.hold()

    def calculateik(self, pose, n): #A function to determine the best IK solution from a given set n
        realpose = pose #Set the pose variable to ensure no instantiation errors with SE3 objects
        poselist = []
        scoring = []
        for i in range(n): #Create a list of n IK solutions with a random seed to ensure deviation between each solution
            y = self.ikine_LM(Tep = realpose, q0 = self.q, joint_limits = True, seed = random.randint(0,10000))
            poselist.append(y.q)
            scoring.append(self._determinescore(y.q,1,0.8)) #Create a list of scores for the given solution set, a greater weighting towards the angle change cost
        bestq = poselist[scoring.index(min(scoring))]
        return bestq #Pick the solution with the lowest cost and return it
    
    def goto(self, t: SE3, steps, n, gripper = None): #A reusable function to move the end effector to a given position specified by an SE3 pose
        q_goal = self.calculateik(t, n) #Get best IK solution
        qtraj = rtb.jtraj(self.q, q_goal, steps).q #Calculate the join trajectory between the two poses
        if gripper is None:
            for q in qtraj: #Iterate through the pose list and step the environment accordingly
                if self.EStop == False:
                    self.q = q
                    self.environ.step()
                    time.sleep(0.02)
        else:
            gtraj = self.activegripper.traj(gripper, steps)
            for i in range(len(qtraj)): #Iterate through the pose list and step the environment accordingly
                if self.EStop == False:
                    self.q = qtraj[i]
                    self.activegripper.q = gtraj[i]
                    self.activegripper.finger.q = gtraj[i]
                    self.environ.step()
                    time.sleep(0.02)
    
    def _apply_3dmodel(self):
        """
        Collect the corresponding 3D model for each link.\n
        Then compute the relation between the DH transforms for each link and the pose of its corresponding 3D object
        """
        # current_path = os.path.abspath(os.path.dirname(__file__))
        self.links_3d = []
        for i in range(self.n + 1):
            file_name = None
            for ext in ['.stl', '.dae', '.ply']:
                if os.path.exists(os.path.join(self._link3D_dir, self.link3D_names[f'link{i}'] + ext)):
                    file_name = os.path.join(self._link3D_dir, self.link3D_names[f'link{i}'] + ext)
                    break                   
            if file_name is not None:
                if f'color{i}' in self.link3D_names:
                    self.links_3d.append(geometry.Mesh(file_name, color = self.link3D_names[f'color{i}'], collision=True))
                else:
                    self.links_3d.append(geometry.Mesh(file_name, collision=True))
            else:
                raise ImportError(f'Cannot get 3D file at link {i}!')

        link_transforms = self._get_transforms(self._qtest)

        # Get relation matrix between the pose of the DH Link and the pose of the corresponding 3d object
        self._relation_matrices = [np.linalg.inv(link_transforms[i]) @ self._qtest_transforms[i] 
                                   for i in range(len(link_transforms))] 
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":  
    r=UR3E()

    env= swift.Swift()
    env.launch(realtime= True)

    thread1=threading.Thread(target=r.add_to_env(env))
    thread2=threading.Thread(target=r._DHVisualiser())

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join() 
    env.hold()

    #r.test()
    
    
    

   
    