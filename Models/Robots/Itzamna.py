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
import random
import threading
import sys
sys.path.append('../LibrarySorter')
from Pathfinding.Pathfinding import *

class Collision(Exception):
    pass

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
        self.ts = None
        self.environ = None
        self.shapes = None
        self.EStop = False
        self.task_cur = None
        self.current_node = 0
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'Itzamna', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        
        #Rotate the base to be in the correct orientation for the workspace
        self.base = self.base * SE3.Ry(pi/2) * SE3(-0.15,0,0)
        self.q = qtest
        self.ts = ItzThetaStarPathing(self)
# -----------------------------------------------------------------------------------#    
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        links = []
        links.append(rtb.PrismaticDH(theta = pi/2, a = 0, alpha = pi/2, qlim = [0.3,2.7]))
        links.append(rtb.PrismaticDH(theta = 0, a = 0.167391, alpha = 0, qlim = [-2.1,-0.2]))
        a = [0.5284043, 0.397998, 0.397998, 0, 0]
        d = [0, 0, 0, 0, 0.307999]
        o = [0, 0, 0, pi/2, -pi/2]
        alpha = [0, pi/2, 0, pi/2, pi/2]
        qlim = [[-pi/2, pi/2] for _ in range(4)]
        qlim.append([-pi, pi])
        for i in range(5):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i], offset= o[i])
            links.append(link)
        return links
    
    def update_shapes(self, shapes):
        self.ts.update_shapes(shapes)
        self.shapes = shapes
    
    def test(self):
        env = swift.Swift()
        env.launch(realtime= True)
        self.add_to_env(env)
        q1 = [0.3, 0, 0, 0, 0, 0, 0]
        q2 = [2.69, -1, -pi/4, pi/4, -pi/4, pi/4, pi/2]
        qtraj = jtraj(q1, q2, 100).q
        for q in qtraj:
            self.q = q
            env.step()
            time.sleep(0.02)
        goal = SE3(1,1,1)
        q3 = self.ik_solve(goal, 10)
        qtraj = jtraj(self.q, q3, 50).q
        for q in qtraj:
            self.q = q
            env.step()
            time.sleep(0.02)
        self.q = [0, 0, 0, 0, 0, 0, 0]
        self.goto(SE3(1,1,1))
        env.hold()

    def add_to_env(self, env):
        super().add_to_env(env)
        self.environ = env

    def goto(self, pos, precision = 1, threadnum = 8, accuracy = 10, mask = False):
        """
        Sends the robot to the given position avoiding any objects in it's way by implementing an A* algorithm
        _____________________________________________________________________________________________________________
        \npos: Position to send robot too.
        \nprecision: Precision to use for Theta* algorithm, default is .5 (for lower values use a higher number of threads)
        \nthreadnum: Maximum number of threads that can be used for collision checking during Theta* algorithm
        \nsteps: Number of steps to take between each node
        \naccuracy: Number of IK solutions to calculate before deciding on lowest cost
        \nmask: Should orientation be masked off in the IK calculation
        
        \n1. Implement Theta* pathing algorithm
        \n  1.a At each node runs an IK solve using the previous nodes pose and then uses the robot's blockout model to determine collisions with the surrounding environment and whether or not the node is valid
        \n2. Apply smoothing algorithm
        \n  2.a Iterates through all nodes. If a node changes direction within 3 nodes of the current, then deletes the midde nodes
        \n3. Calculate IK Solves for each node along smoothed path
        \n4. Generate trajectories between nodes
        \n5. Animate (With active collision checking)
        """
        if type(pos) == SE3:
            p = (pos.t[0], pos.t[1], pos.t[2])
        path = self.ts.refined_theta_star(goal = p, max_threads = threadnum, step_size = precision)
        if self.shapes is not None:
            interruption = threading.Thread(target = self.check_path_interruption(path))
            interruption.start()
        for i in range(len(path)):
            self.current_node = i
            start = self.fkine(self.q)
            se = SE3(path[i][0], path[i][1], path[i][2])
            steps = self.step_scaling(start, se)
            pose = self.ik_solve(se, accuracy, mask)
            qtraj = jtraj(self.q, pose, steps).q
            self.animate(qtraj)

    def check_path_interruption(self, path: list):
        while True:
            last_node = None
            for i in range(self.current_node,len(path)):
                node = path[i]
                if last_node is not None:
                    distance = np.linalg.norm(np.array(node-last_node))
                    steps = 1
                    if distance > 0.1:
                        steps = distance*10
                    q1 = self.ik_solve(node, 1, True)  
                    q2 = self.ik_solve(last_node, 1, True)              
                    qtraj = jtraj(q1, q2, steps).q
                    for q in qtraj:
                        for shape in self.shapes:
                            if self.iscollided(shape, q):
                                return True
                last_node = node
            return False
                    

    def animate(self, qtraj):
        try:
            for q in qtraj:
                    self.q = q
                    self.environ.step()
                    if self.shapes is not None:
                        for shape in self.shapes:
                            if self.iscollided(shape):
                                raise Collision
                    time.sleep(0.02)
        except Collision:
            self.EStop = True

    def step_scaling(self, node1, node2):
        if type(node1) == SE3:
            node1 = (node1.t[0], node1.t[1], node1.t[2])
        if type(node2) == SE3:
            node2 = (node2.t[0], node2.t[1], node2.t[2])
        distance = np.linalg.norm(np.array(node1) - np.array(node2))
        steps = abs(int(distance * 40))
        if steps < 10:
            steps = 30
        return steps
    
    def resume_task(self):
        match self.task_cur:
            case 'idle':
                pass #Remain Idle
            case 'grab':
                pass #Recalculate route to above the table where it can grab books 
            case 'place':
                pass #Recalculate route to active shelf node

    def ik_solve(self,pos,n,mask):
        realpose = pos #Set the pose variable to ensure no instantiation errors with SE3 objects
        if type(realpose != SE3):
            realpose = SE3(realpose[0], realpose[1], realpose[2])
        poselist = []
        scoring = []
        for i in range(n): #Create a list of n IK solutions with a random seed to ensure deviation between each solution
            if mask:
                y = self.ikine_LM(Tep = realpose, q0 = self.q, joint_limits = True, mask = [1,1,1,0,0,0], seed = random.randint(0,10000))
            else:
                y = self.ikine_LM(Tep = realpose, q0 = self.q, joint_limits = True, seed = random.randint(0,10000))
            poselist.append(y.q)
            scoring.append(self._determinescore(y.q,1,0.8)) #Create a list of scores for the given solution set, a greater weighting towards the angle change cost
        bestq = poselist[scoring.index(min(scoring))]
        return bestq #Pick the solution with the lowest cost and return it

    def _anglechangescore(self, pose): #A scoring algorithm to determine an IK solutions angle change cost
        scorelist = []
        for i in range(len(self.q)):
            scorelist.append(abs(self.q[i]-pose[i])) #Subtract the proposed pose by the IK solver from the current pose to determine change in angle
        x = sum(scorelist) / len(scorelist) # Take the average change in angle to determine cost
        return x #A lower cost is better because it signifies less interpolation from one pose to another

    def _maximisationscore(self, pose): #A scoring algorithm to determine an IK solutions maximisation cost
        x = sum(abs(angle) for angle in pose) / len(pose) #Average angle in final solution
        return x #A score closer to 0 means the robot is more stretched out which is favourable to avoid clipping concerns
            
    def _determinescore(self, pose, x, y): #A function that determines a score with a and b as weighting to determine each scores signifigance, generall, angle change will be more important
        a = self._anglechangescore(pose)
        b = self._maximisationscore(pose)
        return (a*x+b*y)

    def iscollided(self, object, pose = None):
        formerpose = self.q
        if pose != None:
            self.q = pose
        for l in self.links_3d:
                d, _, _ = l.closest_point(object)
                if d is not None and d <= 0:
                    self.q = formerpose
                    return True
        self.q = formerpose
        return False
    
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