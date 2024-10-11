import swift
import os
import time
from roboticstoolbox import *
from ir_support.robots.DHRobot3D import DHRobot3D
import spatialmath.base as spb
from spatialmath import SE3
from math import pi

class LibraryBot(DHRobot3D):
    def __init__(self):    
        """ 
        Robot for sorting library books
        """       
        # DH links
        links = self._create_DH()     

        #Create difctionary of model names
        link3D_names = dict(link0 = 'base_rail',
                            link1 = 'secondary_rail',
                            link2 = 'gantry_1', 
                            link3 = 'elbow_1',
                            link4 = 'elbow_2',
                            link5 = 'elbow_3',
                            link6 = 'elbow_4',
                            link7 = 'wrist_1')

        #Set default transforms of models
        qtest = [0, 0, 0, 0, 0, 0, 0]
        qtest_transforms = [spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0),
                            spb.transl(0, 0, 0)]
        
        #Preset some base variables BEFORE passing the necessary items to the superclass initialisation to avoid them being destroyed
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'LibraryBot', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        
        #Rotate the base to be in the correct orientation for the workspace\
        self.q = qtest
# -----------------------------------------------------------------------------------#    
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        links = []
        links.append(PrismaticMDH(theta = pi, a = 0, alpha = pi/2, qlim = [-1,1]))
        links.append(PrismaticMDH(theta = 100, a = 100, alpha = 100, offset = 100, qlim = [-1,1]))
        a = [0, 0, 0, 0, 0]
        d = [0, 0, 0, 0, 0]
        alpha = [0, 0, 0, 0, 0] #Create the base rotational offset
        qlim = [[-2*pi, 2*pi] for _ in range(6)] #Create the base joint limits for the robot
        for i in range(5):
            link = RevoluteMDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i]) #Create the revolving joints and parse them to a list for the super initialiser
            links.append(link)
        return links
    
    def test(self):
        env = swift.Swift()
        env.launch(realtime= True)
        self.add_to_env(env)
        q1 = [0, 0, 0, 0, 0, 0, 0]
        q2 = [1, 0, 0, 0, 0, 0, 0]
        qtraj = jtraj(q1, q2, 50).q
        for q in qtraj:
            self.q = q
            env.step()
            time.sleep(0.02)
        env.hold()

    def add_to_env(self, env):
        super().add_to_env(env)

if __name__ == "__main__":
    r = LibraryBot()
    r.test()