import swift
import os
import time
from roboticstoolbox import *
import roboticstoolbox as rtb
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
        link3D_names = dict(link0 = 'base_rail_b',
                            link1 = 'secondary_rail_b',
                            link2 = 'gantry_1_b', 
                            link3 = 'elbow_1_b',
                            link4 = 'elbow_2_b',
                            link5 = 'elbow_3_b',
                            link6 = 'elbow_4_b',
                            link7 = 'wrist_1_b')

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
        super().__init__(links, link3D_names, name = 'LibraryBot', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        
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
    
    def test(self):
        env = swift.Swift()
        env.launch(realtime= True)
        self.add_to_env(env)
        q1 = [0, 0, 0, 0, 0, 0, 0]
        q2 = [1, -1, -pi/2, pi/2, -pi/2, pi/2, pi]
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
    # fig = r.plot(r.q)
    # fig.hold()