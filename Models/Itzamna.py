import swift
import os
import time
from roboticstoolbox import *
import roboticstoolbox as rtb
from ir_support.robots.DHRobot3D import DHRobot3D
import spatialmath.base as spb
from spatialmath import SE3
import spatialgeometry as geometry
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import numpy as np
from math import pi

class Itzamna(DHRobot3D):
    def __init__(self):    
        """ 
        Robot for sorting library books
        """       
        # DH links
        links = self._create_DH()     
        #Create difctionary of model names
        link3D_names = dict(link0 = 'base_rail_stl', color0 = (0.2,0.2,0.2,1),
                            link1 = 'secondary_rail_stl', color1 = (0.4,0.4,0.9,1),
                            link2 = 'gantry_1_stl',  color2 = (0.2,0.2,0.2,1),
                            link3 = 'elbow_1_stl', color3 = (0.9,0.9,0.4,1),
                            link4 = 'elbow_2_stl', color4 = (0.2,0.2,0.2,1),
                            link5 = 'elbow_3_stl', color5 = (0.9,0.9,0.4,1),
                            link6 = 'elbow_4_stl', color6 = (0.2,0.2,0.2,1),
                            link7 = 'wrist_1_stl', color7 = (0.9,0.9,0.4,1))
        
        #Set default transforms of models
        qtest = [0, 0, 0, 0, 0, 0, 0]
        qtest_transforms = [spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0)] #All zero because blender models were exported with correct global transforms
        
        #Preset some base variables BEFORE passing the necessary items to the superclass initialisation to avoid them being destroyed
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'Itzamna', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        
        #Rotate the base to be in the correct orientation for the workspace
        self.base = self.base * SE3.Ry(pi/2) * SE3(-0.15,0,0)
        self.q = qtest
# -----------------------------------------------------------------------------------#    
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        links = []
        links.append(rtb.PrismaticDH(theta = pi/2, a = 0, alpha = pi/2, qlim = [-1,1]))
        links.append(rtb.PrismaticDH(theta = 0, a = 0.167391, alpha = 0, qlim = [-1,1]))
        a = [0.5284043, 0.397998, 0.397998, 0, 0]
        d = [0, 0, 0, 0, 0.307999]
        o = [0, 0, 0, pi/2, -pi/2]
        alpha = [0, pi/2, 0, pi/2, pi/2]
        qlim = [[-2*pi, 2*pi] for _ in range(6)]
        for i in range(5):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i], offset= o[i])
            links.append(link)
        return links
    
    def _create_blockout_collision_model_test(self, env):
        links = r.fkine_all(r.q).t  # This returns an SE3 object
        n = 0.2
        for i in range(2, len(links)-1):
            p1 = links[i]
            p2 = links[i+1]
            distance = np.linalg.norm(p1 - p2) + 0.1
            if i == (len(links)-2):
                midpoint = (p1 + p2)/2
            else:
                midpoint = (p1 + p2)/2
            vector = p1 - p2
            angle_x = np.arctan2(vector[1], vector[0])
            angle_y = np.arctan2(vector[2], vector[0])
            angle_z = np.arctan2(vector[2], vector[1])
            t = geometry.Cuboid([distance,n,n], pose = SE3(midpoint[0],midpoint[1],midpoint[2]))
            #t.T = t.T * SE3.rpy([angle_x,angle_y,angle_z],unit='rad',order='xyz')
            t.T = t.T * SE3.Rx(angle_z)
            t.T = t.T * SE3.Ry(angle_y)
            t.T = t.T * SE3.Rz(angle_x)
            env.add(t)
            env.step()
    
    def check_collision(self):
        pass #Function that will create a bounding box around each segment of the robot individually and check collision
    
    def test(self):
        env = swift.Swift()
        env.launch(realtime= True)
        self.add_to_env(env)
        q1 = [0, 0, 0, 0, 0, 0, 0]
        q2 = [1, -1, -pi/4, pi/4, -pi/4, pi/4, pi/2]
        qtraj = jtraj(q1, q2, 50).q
        for q in qtraj:
            self.q = q
            env.step()
            time.sleep(0.02)
        self._create_blockout_collision_model_test(env)
        env.hold()

    def add_to_env(self, env):
        super().add_to_env(env)

    def goto(self, pos, precision, threadnum, steps, accuracy):
        """
        Sends the robot to the given position avoiding any objects in it's way by implementing an A* algorithm
        _____________________________________________________________________________________________________________
        \npos: Position to send robot too.
        \nprecision: Precision to use for A* algorithm, default is .1 (for lower values use a higher number of threads)
        \nthreadnum: Maximum number of threads that can be used for collision checking during A* algorithm
        \nsteps: Number of steps to take between each node
        \naccuracy: Number of IK solutions to calculate before deciding on lowest cost
        
        """
        pass #Function that will implement A* pathfinding with an elispoid collision check

    # def iscollided(self, object):
    #     pass #Function that will check if 

if __name__ == "__main__":
    r = Itzamna()
    r.test()
    # fig = r.plot(r.q)
    # fig2 = plt.figure()
    # ax = fig2.add_subplot(111, projection='3d')
    # p = r.fkine_all(r.q).t
    # for l in p:
    #     plt.scatter(l[0],l[1],[2])
    # plt.show()
    # fig.hold()