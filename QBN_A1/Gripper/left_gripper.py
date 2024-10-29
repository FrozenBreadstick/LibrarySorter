
import swift
import spatialgeometry as geometry
import numpy as np
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
from roboticstoolbox import DHLink, DHRobot, jtraj

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class GripperLeft(DHRobot3D):
      
    def __init__(self):         
        # DH links
        links = self._create_DH()     
        # Names of the robot link files in the directory
        link3D_names = dict(
                            link0 = 'Gripper_base',
                            link1 = 'Gripper_finger',
                            link2 = 'empty_stl'
        )

        # A joint config and the 3D object transforms to match that config
        qtest = np.zeros(self.n_links)
        print(qtest)
        
        qtest_transforms = [spb.transl(0,0,0),    
                            spb.transl(0,0,0),
                            spb.transl(0,0,0),
        ]                                          
                    
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'GripperLeft', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.q = qtest
         #??
        self.engage_flag = False

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model
        """

        links = []

        a = [0.070594,0.0306,0]
        # a = [0.0545,-0.03345,-0.011966,0]
        d=  [0,0,0]
        # d = [0,0,0.0213,0]
        alpha = [0,0,0]
        offset = [0,0,0]

        self.n_links = 3
        n_links = self.n_links
        
        qlim = [[-2*pi,2*pi] for _ in range(n_links)]

        for i in range(n_links):
            link = rtb.RevoluteDH(d= d[i], a=a[i], alpha= alpha[i], qlim= qlim[i], offset = offset[i])
            links.append(link)


        return links
               
    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement,
        while plotting end effector positions
        """
        env = swift.Swift()
        env.launch(realtime= True)      
        self.q = self._qtest  
      
        self.add_to_env(env)
    
        env.hold() 
  






    

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":  
    r = GripperLeft()
    r.test()


    
