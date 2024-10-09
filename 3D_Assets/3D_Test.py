import swift
import os
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
        qtest = [0,0,-pi/2,0,0,0,0]
        qtest_transforms = [spb.transl(0,0,0),
                            spb.trotx(-pi/2),
                            spb.transl(0, 0.15239, 0) @ spb.rpy2tr(pi,pi,pi/2, order='xyz'),
                            spb.transl(0, 0.1524, -0.12) @ spb.rpy2tr(pi,pi,pi/2, order='xyz'),
                            spb.transl(0, 0.39583, -0.027115) @ spb.rpy2tr(pi,pi,pi/2, order='xyz'),
                            spb.transl(0, 0.60903, -0.027316) @ spb.rpy2tr(pi,-pi/2,pi/2, order = 'xyz'),
                            spb.transl(0.000389, 0.60902, -0.11253) @ spb.rpy2tr(pi,-pi/2,-pi/2, order = 'xyz'),
                            spb.transl(-0.083765, 0.61096, -0.11333) @ spb.rpy2tr(pi,0,-pi/2, order = 'xyz')]
        
        #Preset some base variables BEFORE passing the necessary items to the superclass initialisation to avoid them being destroyed
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'LibraryBot', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        
        #Rotate the base to be in the correct orientation for the workspace
        #self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2)
        self.q = qtest
# -----------------------------------------------------------------------------------#    
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        links = [PrismaticDH(theta= pi, a= 0, alpha= pi/2, qlim= [-0.8,0])] # Create a sliding rail link for the slider base
        a = [0, 0.24365, 0.21325, -0.00395, 0.004, 0] #Create a list of offsets A and D for the revolving joints of the rest of the robot
        d = [0.1519, 0, 0, -0.11295, 0.08465, 0.0919]
        alpha = [-pi/2, -pi, pi, -pi/2, -pi/2, 0] #Create the base rotational offset
        qlim = [[-2*pi, 2*pi] for _ in range(6)] #Create the base joint limits for the robot
        for i in range(6):
            link = RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i]) #Create the revolving joints and parse them to a list for the super initialiser
            links.append(link)
        return links
    
    def test(self):
        env = swift.Swift()
        env.launch(realtime= True)
        self.add_to_env(env)

    def add_to_env(self, env):
        super().add_to_env(env)

if __name__ == "__main__":
    r = LibraryBot()
    r.test()