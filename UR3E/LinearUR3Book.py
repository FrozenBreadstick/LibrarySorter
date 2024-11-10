from ir_support import *
import swift
import numpy as np
import spatialgeometry as geometry
from spatialmath import *
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import logging
import time
import random
import math
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import os
from math import pi

logging.basicConfig( level=logging.INFO, format='%(asctime)s - %(message)s', handlers=[logging.FileHandler("execution_log.log"), logging.StreamHandler()])


# -----------------------------------------------------------------------------------#
class LinearUR3(DHRobot3D):   
    def __init__(self):    
        """ 
        UR3 Robot on a Linear Rail.
        See the use of `UR3`, `UR5` and base class `DHRobot3D`

        Reach Limits:
        X = 0.48, -1.28, Y = +-0.048 , Z = 
        """       
        self.EStop = False
        # DH links
        links = self._create_DH()     

        #Create difctionary of model names
        link3D_names = dict(link0 = 'base_rail_s', color0 = (0.2,0.2,0.2,1),
                            link1 = 'slider_rail_s',
                            link2 = 'shoulder_ur3_s',
                            link3 = 'upperarm_ur3_s',
                            link4 = 'forearm_ur3_s',
                            link5 = 'wrist1_ur3_s',
                            link6 = 'wrist2_ur3_s',
                            link7 = 'wrist3_ur3_s')

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
        self.activegripper = None
        self.activebrick = None
        self.environ = None
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'LinearUR3', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        
        #Rotate the base to be in the correct orientation for the workspace
        self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2)
        self.q = qtest


        
    # -----------------------------------------------------------------------------------#    
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        links = [rtb.PrismaticDH(theta= pi, a= 0, alpha= pi/2, qlim= [-0.8,0])] # Create a sliding rail link for the slider base
        a = [0, 0.24365, 0.21325, -0.00395, 0.004, 0] #Create a list of offsets A and D for the revolving joints of the rest of the robot
        d = [0.1519, 0, 0, -0.11295, 0.08465, 0.0919]
        alpha = [-pi/2, -pi, pi, -pi/2, -pi/2, 0] #Create the base rotational offset
        qlim = [[-pi, pi] for _ in range(6)] #Create the base joint limits for the robot
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i]) #Create the revolving joints and parse them to a list for the super initialiser
            links.append(link)
        return links
                    
    # -----------------------------------------------------------------------------------#
    #Logging Function
    def eeplog(self):
        logging.info(f"End-effector pose: \n{self.fkine(self.q)}")
    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Demonstrate maximum reach of robot, and calculate aproximate workspace volume
        """
        logging.info("Running test function")
        env = swift.Swift()
        env.launch(realtime= True) #Create environment for test
        self.q = self._qtest
        fig = plt.figure()
        ax = plt.axes(projection ='3d') #Create plot objects and axes in a 3D projection for plotting trajectory
        self.add_to_env(env) #Add the LinearUR3 to the scene
        p = [] #List to store important points
        points = [] #List to store all points
        self.q = [0,0,0,0,-pi/2,0,0] #Starting pose
        p.append(self.fkine(self.q).t) #Addd first important point
        time.sleep(1)
        #List of all points to translate to during test
        q_goals = [[-0.8,0,0,0,-pi/2,0,0], [-0.8,-pi,0,0,-pi/2,0,0], [0,-pi,0,0,-pi/2,0,0], [0,-3*pi/2,0,0,-pi/2,0,0], [0,-3*pi/2,-pi/2,0,-pi/2,0,0], [-0.8,-3*pi/2,-pi/2,0,-pi/2,0,0], [-0.8,-3*pi/2,-pi,0,-pi/2,0,0], [-0.8,-3*pi/2,-3*pi/2,0,-pi/2,0,0]]
        for goal in q_goals: #Animate the robot going to each joint position
            time.sleep(0.5)
            qtraj = rtb.jtraj(self.q, goal, 50).q
            for q in qtraj:
                self.q = q
                env.step(0.02)
                points.append(self.fkine(self.q).t) #Save every point for plotting later
            p.append(self.fkine(self.q).t) #Save the important points (the position at the start of each new translation)
            self.eeplog()
        translationallen = math.sqrt((p[0][0] - p[1][0])**2 + (p[0][1] - p[1][1])**2 + (p[0][2] - p[1][2])**2) #Mathematical calculations regarding workspace size and bounds
        radius = (abs(p[0][1]) + abs(p[1][1]) + abs(p[2][1]) + abs(p[3][1]) + abs(p[4][0]))/5
        circle = (pi*(radius**2))
        vol1 = circle*translationallen
        vol2 = vol1 + pi*((radius*2)**2)
        x = [points[i][0] for i in range(len(points))]
        y = [points[i][1] for i in range(len(points))]
        z = [points[i][2] for i in range(len(points))]
        print("Workspace Volume is: " + str(vol2))
        bounds = [(max(x), min(x)) for x in zip(*points)]
        print("Workspace X bounds are: (" + str(round(bounds[0][0], 4)) + "," + str(round(bounds[0][1], 4)) + ")")
        print("Workspace Y bounds are: (" + str(round(bounds[1][0], 4)) + "," + str(round(bounds[1][1], 4)) + ")")
        print("Workspace Z bounds are: (" + str(round(bounds[2][0], 4)) + "," + str(round(bounds[2][1], 4)) + ")")
        ax.plot(x, y, z, color ='green') #Plot the motion of the robot
        ax.set_title('Plot of Position')
        plt.show()
        time.sleep(3)
        input("Press Enter!")
        env.close()
    # -----------------------------------------------------------------------------------#

    def add_to_env(self, env): #Override the superclass environment adder function
        super().add_to_env(env) #Use the superclass method to add itself to the environment
        self.environ = env #Set environment variable for later

    #Gripper Functions
    @property #Override the superclass q property, allowing it to be modified ourself
    def q(self):
        return self._q

    @q.setter #Override the superclass q setter allowing it to be modified
    def q(self, new_q):
        self._q = new_q
        self.fkine(self._q)

        # If there is an active gripper, update its base
        if self.activegripper is not None:
            end_effector_pose = self.fkine(self._q).A * SE3.Rx(pi/2) * SE3.Ry(pi/2) * SE3.Rz(pi) * SE3(0,0.088,0) #Apply an offset to position the gripper correctly
            self.activegripper.base = SE3(end_effector_pose) #Update the base of the gripper with the movement of the end effector

        #If there is an active Brick, update it's base
        if self.activebrick is not None:
            end_effector_pose = self.fkine(self._q).A * SE3(0,0,-0.32) #Apply an offset to ensure it is position correctly
            self.activebrick.brick.T = SE3(end_effector_pose) #Update the base of the gripper with the movement of the end effector

    def attachgripper(self, gripper):
        self.activegripper = gripper
        #Initialize the gripper base to match the robot's base
        if self.activegripper is not None:
            self.activegripper.base = self.base * SE3.Ry(pi)

    # -----------------------------------------------------------------------------------#
    #Inverse Kinematic Functions
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
    
    def calculateik(self, pose, n, mask = False): #A function to determine the best IK solution from a given set n
        realpose = pose #Set the pose variable to ensure no instantiation errors with SE3 objects
        poselist = []
        scoring = []
        logging.info(f"Calculating {n} IK solutions for pose: \n{pose}")
        for i in range(n): #Create a list of n IK solutions with a random seed to ensure deviation between each solution
            if mask == False:
                y = self.ikine_LM(Tep = realpose, q0 = self.q, joint_limits = True, seed = random.randint(0,10000))
            else:
                y = self.ikine_LM(Tep = realpose, q0 = self.q, joint_limits = True, seed = random.randint(0,10000), mask=[1,1,1,0,0,0])
            poselist.append(y.q)
            scoring.append(self._determinescore(y.q,1,0.8)) #Create a list of scores for the given solution set, a greater weighting towards the angle change cost
        bestq = poselist[scoring.index(min(scoring))]
        logging.info(f"Found lowest cost solution as: \n{bestq}")
        return bestq #Pick the solution with the lowest cost and return it
    
    def goto(self, t: SE3, steps, n, gripper = None, book = None, mask = False): #A reusable function to move the end effector to a given position specified by an SE3 pose
        q_goal = self.calculateik(t, n, mask) #Get best IK solution
        qtraj = rtb.jtraj(self.q, q_goal, steps).q #Calculate the join trajectory between the two poses
        self.eeplog()
        logging.info("moving to:")
        if gripper is None:
            for q in qtraj: #Iterate through the pose list and step the environment accordingly
                if self.EStop == False:
                    self.q = q
                    if book != None:
                        book.T = self.fkine(self.q)
                    self.environ.step()
                    # time.sleep(0.02)
            self.eeplog()
        else:
            gtraj = self.activegripper.traj(gripper, steps)
            for i in range(len(qtraj)): #Iterate through the pose list and step the environment accordingly
                if self.EStop == False:
                    self.q = qtraj[i]
                    self.activegripper.q = gtraj[i]
                    self.activegripper.finger.q = gtraj[i]
                    self.environ.step()
                    # time.sleep(0.02)
            self.eeplog()

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

    def alignxyz(self, t: SE3, n, gripper = None): #An alternative to the goto function that seperates that interpolation between two poses into X, Y, and Z steps
        if type(t) != SE3:
            t = SE3(t) #ensure t is an SE3 pose and not a numpy array
        rot = t.rpy() #extract the rotation
        # Align X
        s = self.fkine(self.q) #Determine current joint state to edit individual components
        s = SE3(t.t[0], s.t[1], 0.5) * SE3.Rz(rot[2]) #A mixture of components from our current joint state, and the target joint state
        self.goto(s,100,n, gripper) #Call goto in order to complete the interpolation
        # Align Y
        s = self.fkine(self.q)
        s = SE3(s.t[0], t.t[1], 0.5) * SE3.Rz(rot[2])
        self.goto(s,100,n, gripper)
        # Align Z
        s = self.fkine(self.q)
        s = SE3(s.t[0], s.t[1], t.t[2]) * SE3.Rz(rot[2]) * SE3.Ry(rot[1]) * SE3.Rx(rot[0])
        self.goto(s,100,n, gripper)

    def pickbrick(self, brick, n, gripper:float = None): #Function to pick up brick from brick's location
        logging.info(f"Picking brick at position: \n{brick.brick.T}")
        #Hover above the brick
        self.goto((brick.brick.T * SE3(0,0,0.4)), 100, n, gripper)

        # Grab the brick
        if gripper is None:
            self.activegripper.open(1)
        self.goto((self.fkine(self.q) * SE3(0,0,-0.08)), 30, n)
        self.activegripper.open(0.6)

        #Attach brick for movement
        self.activebrick = brick
    
    def pickBook(self, booksPose, n, gripper:float = None): #Function to pick up brick from brick's location
        logging.info(f"Picking book at position: \n{booksPose.T}")
        #Hover above the brick
        self.goto((booksPose.T * SE3(0,0,0.4)), 100, n, gripper)

        # Grab the brick
        if gripper is None:
            self.activegripper.open(1)
            
        self.goto((self.fkine(self.q) * SE3(0,0,-0.08)), 30, n)
        self.activegripper.open(0.6)


    def dropbrick(self, pos, n): #Function to drop brick in target position
        logging.info(f"Dropping brick in wall position: \n{pos}")
        #Place brick in drop position, AlignXYZ is ideal as it ensure the bricks do not hit one another while being placed
        self.alignxyz((pos * SE3(0,0,0.315)), n)
        
        # Drop the brick and retract gripper
        self.activegripper.open(1)
        self.activebrick = None
        self.goto((self.fkine(self.q) * SE3(0,0,0.08)), 30, n)

        # Open gripper to pick up next brick
        self.activegripper.open(0)

    def resetpos(self, n): #A function to bring the arm back to a common outstretched state to avoid crumpling while grabbin bricks
        logging.info("Returning to reset pose...")
        pos = self.fkine(self.q)
        pos.t[2] = 0.5 #Raise arm in current position
        self.goto(pos, 30, 20)

        pos = self.fkine(self.q)
        pos.t[1] = 0
        pos.t[0] = 0.4 #Interpolate to a position past the rail bounds such that the arm is stretched out a fair way
        self.goto(pos, 50, 20)

        q_goal = [self.q[i] for i in range(self.n)]
        q_goal[0] = -0.8 #Travel along rails to furthest point in order to keep outstretched pose during next IK calculation
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        for q in qtraj:
            self.q = q
            self.environ.step()
            time.sleep(0.02)
# -----------------------------------------------------------------------------------#
class GripFinger(DHRobot3D): #A robot entity that is an empty base with a singular prismatic slider joint.
    def __init__(self):
        """
        Gripper Finger
        """
        links = self._create_DH()

        link3D_names = dict(link0= 'Empty', #Empty base model
                            link1 = 'gripper_finger_b') #Gripper finger model
        
        qtest = [0]
        qtest_transforms = [spb.transl(0,0,0) @ spb.troty(pi/2),
                            spb.transl(0,0,0) @ spb.troty(pi/2)]
        
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'Grip', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.base = self.base * SE3.Ry(-pi/2) * SE3.Rz(pi/2) #Rotate the base so that it aligns
        self.q = qtest

    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        links = [rtb.PrismaticDH(theta= pi, a= 0, alpha= pi/2, qlim= [-0.1,0.1])] #Create prismatic link
        return links

class Grip(DHRobot3D):   #The main griper object class
    def __init__(self):    
        """ 
        Gripper
        """       
        # DH links
        links = self._create_DH()     

        link3D_names = dict(link0 = 'gripper_base_b',#Create gripper base model
                            link1 = 'gripper_finger_b') #Creat gripper finger
        
        qtest = [0]
        qtest_transforms = [spb.transl(0,0,0) @ spb.troty(pi/2),
                            spb.transl(0,0,0) @ spb.troty(pi/2)]
        
        self._base = SE3()  #Initialize _base to an identity transformation
        self.finger = GripFinger() #Instatiate a second finger
        self.environ = None
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'Grip', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.base = self.base * SE3.Ry(pi/2) * SE3.Rz(pi/2)
        self.q = qtest
        
    # -----------------------------------------------------------------------------------#    
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        links = [rtb.PrismaticDH(theta= pi, a= 0, alpha= pi/2, qlim= [-0.1,0.1])]
        return links
        
                    
    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        self.add_to_env(env)
        
        self.q = self._qtest
        q_goal = [0.05]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        
        for q in qtraj:
            self.q = q
            self.finger.q = q 
            self.base = self.base * SE3(0,0,0.01)
            env.step(0.02)
            time.sleep(0.02)

    # -----------------------------------------------------------------------------------#
    def add_to_env(self, env): #Override the superclass environment adder function
        """
        Add both the Grip and the GripperFinger to the environment
        """
        super().add_to_env(env) #Use the superclass method to add itself to the environment
        self.finger.add_to_env(env) #Given GripFinger does not override this function, it can be called normally
        self.environ = env

    # -----------------------------------------------------------------------------------#
    def open(self, amount: float): #A method for opening the gripper to a percentage amount
        """
        amount: A float between 0 and 1 that specifies the percentage 'open' the gripper should be
        """
        q_goal = [amount * 0.05] #0.05 is the maximum amount that the gripper can be opened before it slides out of it's rail housing
        qtraj = rtb.jtraj(self.q, q_goal, 20).q

        for q in qtraj:
            self.q = q
            self.finger.q = q  #Sync the GripperFinger movement with the Grip movement
            self.environ.step(0.02)
            time.sleep(0.02)
    
    def traj(self, amount:float, steps):
        q_goal = [amount * 0.05] #0.05 is the maximum amount that the gripper can be opened before it slides out of it's rail housing
        qtraj = rtb.jtraj(self.q, q_goal, steps).q
        return qtraj

    # -----------------------------------------------------------------------------------#
    def _fingerbase(self): #Create a function to set the GripFinger object pairing's base, and multiply it by 180 degrees such that it mirrors the Grip objects fingers
        self.finger.base = self.base * SE3.Ry(pi)

    @property #Override the base property so we can modify it ourselves
    def base(self):
        return self._base

    @base.setter #Override the base setter to set both the base of the Grip object and the GripFinger object
    def base(self, new_base):
        self._base = new_base
        self._fingerbase()
    # -----------------------------------------------------------------------------------#

class EndEffectorControl:
    def __init__(self, root, r):
        self.root = root #Parse the root window and set the variable for window object creation 
        self.root.title("Set End Effector Pose")
        self.r = r

        self.x_label = ttk.Label(root, text="X (0.579,-1.3782):") #Create an X input field
        self.x_label.grid(row=0, column=0, padx=10, pady=10)
        self.x_entry = ttk.Entry(root)
        self.x_entry.grid(row=0, column=1, padx=10, pady=10)

        self.label_y = ttk.Label(root, text="Y (0.5789,-0.5416):") #Create a Y input field
        self.label_y.grid(row=1, column=0, padx=10, pady=10)
        self.y_entry = ttk.Entry(root)
        self.y_entry.grid(row=1, column=1, padx=10, pady=10)

        self.z_label = ttk.Label(root, text="Z (0.6935,-0.3897):") #Create a Z input field
        self.z_label.grid(row=2, column=0, padx=10, pady=10)
        self.z_entry = ttk.Entry(root)
        self.z_entry.grid(row=2, column=1, padx=10, pady=10)

        self.pos_label = ttk.Label(root, text="End Effector: (0.0, 0.0, 0.0)") #Create a label that show the current end effector translation position
        self.pos_label.grid(row=3, column=0, columnspan=2, pady=10)

        self.submit_button = ttk.Button(root, text="Submit", command=self.submit) #Submit button
        self.submit_button.grid(row=4, column=0, columnspan=1, pady=20)
        self.getpos_button = ttk.Button(root, text="Get Pos", command=self.getpos) #Button to return end effector position
        self.getpos_button.grid(row=4, column=1, columnspan=2, pady=20)

    def submit(self):
        try:
            x = float(self.x_entry.get()) #Get the data from each entry box
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
            if (0.579 <= x <= -1.3782):
                raise ValueError("X value out of bounds.") #Raise an error if the value is out of bounds of the robot's reach
            if (0.5789 <= y <= -0.5416):
                raise ValueError("Y value out of bounds.")
            if (0.6935 <= z <= -0.3897):
                raise ValueError("Z value out of bounds.")
            self.r.goto(SE3(x,y,z), 100, 20)
        except ValueError as e:
            messagebox.showerror("Input Error", str(e)) #Display a messagebox with the error if error is raised
    
    def getpos(self):
        position = self.r.fkine(self.r.q).t #Use forward kinematics to get the end effector's position
        self.pos_label.config(text=f"End Effector: ({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f})") #Update the label information

class Brick(): #A brick class for instatiating brick objects
    def __init__(self, t: SE3):
        self.brick = geometry.Mesh('brick.dae')
        self.brick.T = t
    
    def add_to_env(self, environ):
        environ.add(self.brick)

def brickwallsetup(env): #Setup for the for the bricks starting position and the positions for the bricks to be placed in a 3x3 wall config
    global bricks
    global wall #Ensure brick and wall lists can be used later
    bricks = [Brick(SE3(SE3(0.15 + -0.1355 * (i//3),-0.3,(i%3)*0.0333) * SE3.Rz(np.pi/2))) for i in range(9)] #Systematically place bricks in wall configuration
    wall = [brick.brick.T for brick in bricks] #Save wall positions
    for i in range(9):
        bricks[i].brick.T = (SE3(SE3(-0.15 * (i%3),0.25 + (0.11*(i//3)),0) * SE3.Rz(np.pi/2))) #Change bricks location to pick up locations
    for brick in bricks:
        brick.add_to_env(env) #Add all bricks into the environment

def railsetup(r, env):
    global rails
    rails = [geometry.Mesh('railing.dae') for i in range(4)]
    rails[0].T = r.base * SE3(0,0,0.65)
    rails[1].T = r.base * SE3(0.8,0,-0.4) * SE3.Ry(np.pi/2)
    rails[2].T = r.base * SE3(-0.8,0,-0.4) * SE3.Ry(np.pi/2)
    rails[3].T = r.base * SE3(0,0,-1.5)
    for rail in rails:
        env.add(rail)

def bricksimulation(r):
    env = swift.Swift() #Launch swift virtual 3D environment
    env.launch()
    joint_angles = np.array(np.zeros(7)) #Set startin position
    joint_angles[0] = -0.8
    joint_angles[2] = -np.pi/2
    r.q = joint_angles

    brickwallsetup(env)
    railsetup(r,env)

    global grippy
    grippy = Grip() #Create gripper and attach it
    r.attachgripper(grippy)

    grippy.add_to_env(env) #Add objects to virtual environment
    r.add_to_env(env)

    for i in range(len(bricks)): #Iterate through the bricks
        r.pickbrick(bricks[i], 20, 1)
        r.resetpos(20)
        r.dropbrick(wall[i], 20)
        r.resetpos(20)
    input("Press Enter!")
    env.close()

def guisetup(r):
    env = swift.Swift() #Launch swift virtual 3D environment
    env.launch()
    r.add_to_env(env)

if __name__ == "__main__":
    
    r = LinearUR3() #Create LinearUR3 robot
    fig = r.plot(r.q)
    fig.hold()
    
    # r.test() #Plot workspace and calculate volume

    bricksimulation(r) #Run brick simulation

    # guisetup(r) #Open Extension GUI component 
    # root = tk.Tk()
    # app = EndEffectorControl(root, r)
    # root.mainloop()