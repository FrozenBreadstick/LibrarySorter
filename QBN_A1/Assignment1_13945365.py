##  @file
#   @brief Main Run file for Lab Assignment 1
#   @author Quoc Binh NGUYEN
#   @date Sep 17, 2024

from UR3E.UR3E import UR3E
from ir_support import UR3
import roboticstoolbox as rtb
from spatialmath import SE3
import spatialmath.base as spb
import numpy as np
import spatialgeometry as geometry
import swift
from math import pi
import math
import time
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import jtraj
from bagpy import bagreader
import pandas as pd
import os

def pickAndPlaceRun():
    '''
    This function demonstrates the pick and place of bricks
    Robot base and bricks locations can be changed and the final pose are still achieved
    '''
    env=swift.Swift()
    env.launch()
    r=UR3E()
    set_safety_env(env)
    #r.base=r.base*SE3(0,0,0.1)
    r.add_to_env(env)

    #Set up environment
    brick_Path='/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/Assignment_1/BRICK.dae'

    #Adjust bricks innit pose
    bricksPosition=[SE3(-0.4,-0.1,0.035),
                    SE3(-0.4,0,0.035),
                    SE3(-0.4,0.1,0.035),
                    SE3(-0.4,0.2,0.035),
                    SE3(-0.4,0.3,0.035),
                    SE3(-0.4,0.4,0.035),
                    SE3(-0.4,0.5,0.035),
                    SE3(-0.4,0.6,0.035),
                    SE3(-0.4,0.7,0.035)]
    bricksRotation=[pi/2,
                    pi/2,
                    pi/2,
                    pi/2,
                    pi/2,
                    pi/2,
                    pi/2,
                    pi/2,
                    pi/2] 
    brickPose=[]
    brickRef=[] #For keeping check of bricks
    
    z_offset=0.035# for gap to make visualisation better
    bricksFinalPosition=[SE3(0.4,0.2,0.035),#Row 1 column 1
                         SE3(0.4,0.35,0.035),#Row 1 column 2
                         SE3(0.4,0.5,0.035),#Row 1 column 3
                         SE3(0.4,0.2,2*0.035+z_offset),#Row 2 column 1
                         SE3(0.4,0.35,2*0.035+z_offset),#Row 2 column 2
                         SE3(0.4,0.5,2*0.035+z_offset),#Row 2 column 3
                         SE3(0.4,0.2,3*0.035+1.85*z_offset),#Row 3 column 1
                         SE3(0.4,0.35,3*0.035+1.85*z_offset),#Row 3 column 2
                         SE3(0.4,0.5,3*0.035+1.85*z_offset)]#Row 3 column 3
    bricksFinalRotation=[0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0]
    brickFinalPose=[]
    
    # Prepare final brick poses
    for i in range(len(bricksFinalPosition)):
        pose=(SE3(bricksFinalPosition[i])@SE3.Rz(bricksFinalRotation[i])).A
        brickFinalPose.append(pose)

    #Add bricks in the environment
    for i in range(len(bricksPosition)):
        pose=(SE3(bricksPosition[i])@SE3.Rz(bricksRotation[i])).A
        brickPose.append(pose)
    
    for i in range(len(brickPose)):
        brickMesh=geometry.Mesh(filename=brick_Path,pose=brickPose[i],color=(0.4,0.04,0.04),scale=[0.04,0.07,0.03])
        brickRef.append(brickMesh)
        env.add(brickMesh)
    
    STEPS=80
    
    #Loop to identify bricks, and pick and place
    for i in range(len(brickPose)):
        #Getting robot current joint configuration
        q0=r.q
        
        q1=r.ikine_LM(brickPose[i],np.zeros(7),joint_limits=True).q
        #Genrate q matrix and taking only position over steps
        q_matrix = jtraj(q0, q1, STEPS).q
        
        #Visualise Locating motion
        for q in q_matrix:
            r.q=q
            env.step(0.02)
        print("Brick " + str(i) + " located")
        time.sleep(1)

        PoseMid=SE3(0.2,0.5,0.5)
        qmid=r.ikine_LM(PoseMid,np.zeros(7),joint_limits=True).q
        qmatrix = jtraj(q1, qmid, STEPS).q
        for q in qmatrix:
            #Assigned r.q = calulated q
            r.q=q
            #Use fkine to find end effector pose with the calculated q
            ee_pose = r.fkine(r.q)
            log_transform_to_file(ee_pose)
            #Assign the end effector pose to the brick
            brickPose[i] = ee_pose
            
            #Remove the old brick
            env.remove(brickRef[i])
            
            #Create new mesh for the same brick with new pose
            newBrickMesh=geometry.Mesh(filename=brick_Path,pose=brickPose[i],color=(0.4,0.04,0.04),scale=[0.04,0.07,0.03])
            #Assign the new mesh to the brick reference
            brickRef[i]=newBrickMesh
            env.add(newBrickMesh)
            env.step(0.02)
            env.step(0.02)

        #Visualise picking motion from innitial position to final position
        q2=r.ikine_LM(brickFinalPose[i],np.zeros(7),joint_limits=True).q
        q_matrix = jtraj(qmid, q2, STEPS).q
        for q in q_matrix:
            #Assigned r.q = calulated q
            r.q=q
            #Use fkine to find end effector pose with the calculated q
            ee_pose = r.fkine(r.q)
            log_transform_to_file(ee_pose)
            #Assign the end effector pose to the brick
            brickPose[i] = ee_pose
            
            #Remove the old brick
            env.remove(brickRef[i])
            
            #Create new mesh for the same brick with new pose
            newBrickMesh=geometry.Mesh(filename=brick_Path,pose=brickPose[i],color=(0.4,0.04,0.04),scale=[0.04,0.07,0.03])
            #Assign the new mesh to the brick reference
            brickRef[i]=newBrickMesh
            env.add(newBrickMesh)
            env.step(0.02)
        print("Brick " + str(i) + " placed") 
    
    print("All bricks placed(@_@)")
    env.hold()

def SafetyEnvRun():
    '''
    This function demonstartes the environment with safety features
    with the robot moving back and forth
    '''
    r = UR3E()
   
    env = swift.Swift()
    env.launch(realtime=True)

    #add objects in the environment
    set_safety_env(env)

    r.add_to_env(env)

    #Manipulate the environment to display the robot movement
    movingDemo(r,env)
    env.hold()

def set_safety_env(env):
    '''
    This function sets the environment
    param env: Specified environment
    '''
    print("Setting up the environment...")

    
    """# Add table in the environment
    table_path='/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/Assignment_1/table.dae'
    table_position = [SE3(0,0.7,0)]
    table_mesh=geometry.Mesh(filename=table_path, pose=table_position[0])
    env.add(table_mesh)
    print("Setting up Table done....") """

    # Add Estop in the environment
    estop_path='/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/Assignment_1/eStop.dae'
    estop_position = [SE3(-1.8,0,1.3),
                      SE3(1.8,0,1.3),]
    estop_Yrotations = [-pi/2,
                       pi/2]
    estop_pose = []

    for i in range(len(estop_position)):
        poseEstop = (SE3(estop_position[i]) * SE3.Ry(estop_Yrotations[i])).A
        estop_pose.append(poseEstop)

    for i in range(len(estop_pose)):
        estop_mesh = geometry.Mesh(filename=estop_path, pose=estop_pose[i])
        env.add(estop_mesh)
    print("Setting up Estops done....")
    
    #Add fences in the environment
    safety_fencing_path = '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/Assignment_1/fencing.dae'
  
    
    fence_positions = [SE3(-1.72, 0, 0),
                                SE3(-1.72, 2, 0),
                                SE3(1.72, 2, 0),
                                SE3(1.72, 0, 0),
                                SE3(0,-1.6,0),
                                SE3(0,3.8,0)] 
    fence_Zrotations = [0,
                                0,
                                0,
                                0,
                                pi/2,
                                pi/2,]    
    # Grow depends on size of fence_position
    fence_pose = []
    for i in range(len(fence_positions)):
        poseFence = (SE3(fence_positions[i]) * SE3.Rz(fence_Zrotations[i])).A
        fence_pose.append(poseFence)
        
    for i in range(len(fence_pose)):
        Fences_mesh = geometry.Mesh(filename=safety_fencing_path, pose=fence_pose[i])
        env.add(Fences_mesh)
    print("Setting up fencings done....")

    #Add Fire Extinguisher in the environment
    exstinguisher_path= '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/Assignment_1/Extinguisher.dae'
    exstinguisher_position = [SE3(1.9,0,0),
                              SE3(-1.9,0,0)]
    exstinguisher_Yrotations = [0,
                                0]
    exstinguisher_pose = []

    for i in range(len(exstinguisher_position)):
        poseExstinguisher = (SE3(exstinguisher_position[i]) * SE3.Ry(exstinguisher_Yrotations[i])).A
        exstinguisher_pose.append(poseExstinguisher)
    
    for i in range(len(exstinguisher_pose)):
        exstinguisher_mesh = geometry.Mesh(filename=exstinguisher_path, pose=exstinguisher_pose[i],scale=[0.002,0.002,0.002])
        env.add(exstinguisher_mesh)
    print("Setting up Extinguisher done....")
    print("Environment setup done....")

def giveEEPoseAndDetermineJointStateAndEvidence():
    '''
    This function gives the end effector pose and determines the joint state for the given end effector pose
    This function will also check for the evidence of the joint state being within tolerance
    '''
    r=UR3E()

    # Desired end-effector position
    EEposition=SE3(0.2,0.5,0.5)
    distance=6/1000
    while distance*1000 > 5:
        # Calculate joint configuration for the given end-effector pose
        q1=r.ikine_LM(EEposition,np.zeros(len(r.q)),joint_limits=True).q
        print("Joint state for the given end effector pose is: ",q1)

        CalculatedEEpose=r.fkine(q1)
        print("Calculated end effector pose for the given joint state is: ",CalculatedEEpose)
        Xdiff=abs(EEposition.t[0]-CalculatedEEpose.t[0])
        Ydiff=abs(EEposition.t[1]-CalculatedEEpose.t[1])
        Zdiff=abs(EEposition.t[2]-CalculatedEEpose.t[2])


        #Calculate the Euclidean distance between the two points
        distance = np.sqrt(Xdiff**2 + Ydiff**2 + Zdiff**2)
        print("Euclidean distance between the requested and calculated end-effector positions in mm: ", distance * 1000)
        

def WorkSpaceReachRadius(r):
    '''
    This function calculates the reach of the robot in the x and y direction of the global frame

    param r: Specified robot
    '''
   
    max_reach_q = np.zeros(len(r.q))
    T = r.fkine(max_reach_q)
    ee_pos_max = T.t
    max_reach = np.linalg.norm(ee_pos_max)
    max_reach = max_reach
    print("max reach (m, x-dir) = ", max_reach)
    print("Radius of x=",max_reach/2)
    print("max reach (m, y-dir) = ", 0.8+ max_reach)
    print("Radius of x=",(0.8+max_reach)/2)

def plotWorkspace(r,angle):
    '''
    This function calculates the workspace of the robot and plots it
    param r: Specified robot
    param angle: angle of the step in degrees
    '''
    step_rads = math.radians(angle)
    qlim = np.transpose(r.qlim) # Transpose to get the 6x2 matrix for each joint as a row
    print("Joint limits: ", qlim)

    # Don't need to worry about joint 6-th
    pointcloud_size = int(np.prod(np.floor((qlim[0:7, 1] - qlim[0:7, 0]) / step_rads + 1)))
    pointcloud = np.zeros((pointcloud_size, 3))
    counter = 0
    start_time = time.time()
    print("Start creating point cloud...")
    for q0 in np.arange(0,-0.8,-0.1):
        for q1 in np.arange(0, qlim[1,1], step_rads):
            for q2 in np.arange(0, qlim[2,1], step_rads):
                for q3 in np.arange(0, qlim[3,1], step_rads):
                    for q4 in np.arange(0, qlim[4,1] , step_rads):
                        for q5 in np.arange(0, qlim[5,1], step_rads):
                            for q6 in np.arange(0, qlim[6,1], step_rads):
                              q7 = 0
                              q = [q0, q1, q2, q3, q4, q5,q6]
                              tr = r.fkine(q).A
                              if counter == pointcloud_size:
                                  break
                              pointcloud[counter,:] = tr[0:3,3]
                              counter += 1
                              if np.mod(counter/pointcloud_size * 100, 1) ==0:
                                  end_time = time.time()
                                  execution_time = end_time - start_time
                                  print(f"After {execution_time} seconds, complete", counter/pointcloud_size*100, "% of pose")
                              
    # 2.6 Create a 3D model showing where the end effector can be over all these samples
    plt.close()
    ax = plotvol3() # Create a 3D plot 
    ax.plot(pointcloud[:,0], pointcloud[:,1], pointcloud[:,2], 'r')
    plt.show()

def log_transform_to_file(ee_pose, filename="transform_log.txt"):
    """
    Logs the transformation matrix of the end-effector to an external file.
    :param ee_pose: SE3 object containing the transformation matrix
    :param filename: The name of the log file
    """
    # Open the file in append mode to keep previous logs
    with open(filename, "a") as log_file:
        log_file.write("End Effector Transformation Matrix:\n")
        log_file.write(np.array2string(ee_pose.A) + "\n")  # Log the transformation matrix as a string
        log_file.write("\n")  # Add a newline for readability


def movingDemo(r,env):
        '''
        This function moves the robot BACK AND FORTH beween two configurations
        param r: Specified robot
        param env: Specified environment
        '''
        for i in range (6):
         q_goal = [r.q[i]-pi/3 for i in range(r.n)]
         q_goal[0]=-0.8  #move rail link
         qtraj = rtb.jtraj(r.q, q_goal, 50).q
         
         for q in qtraj:
             r.q = q
             env.step(0.02)
    
         r.q=q_goal #last position 
         q_goal=r._qtest
         qtraj = rtb.jtraj(r.q, q_goal, 50).q
         for q in qtraj:
             r.q = q
             env.step(0.02)

def read_bag(path): 
    '''
    Function to convert ros .bag files to a text file for processing and analysis.
    :param path: relative path to .bag file.
    '''

    bag = bagreader(path)
    print(bag.topic_table)
    joint_state = bag.message_by_topic('/joint_states')
    joint_data = pd.read_csv(joint_state)
    df = pd.DataFrame(joint_data)
    cols = [6,7,8,9,10,11]  # select columns
    df = df[df.columns[cols]]
    
    output_dir= '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/Assignment_1'
    output_file = os.path.join(output_dir, 'bag_q.txt')
    # Save the DataFrame to a text file
    df.to_csv('bag_q.txt', index=False)  

def move_from_text(path):
    '''
    Function to move the robot to the joint states specified in the text file.
    :param path: relative path to the text file.
    '''
    r = UR3()
    env = swift.Swift()
    env.launch(realtime=True)
    r.add_to_env(env)

    # Read the text file
    moveData = pd.read_csv(path)
    q_matrix=moveData.values
    # Remove the first row of the data
    print(np.shape(q_matrix))
    q_matrix = moveData.values[1:, :]  # Slicing to skip the first row
    print(np.shape(q_matrix))
    
    for i in range(0, len(q_matrix), 10):  # Step of 10 in the range
        
        r.q = q_matrix[i]  # Set the robot joint configuration
        env.step(0.02)
     
     
   

if __name__ == "__main__":
 #Uncomment to demonstrate the safety environment
    #SafetyEnvRun()

 #Uncomment to run the pick and place bricks
    pickAndPlaceRun()


 #Uncomment to calculate the workspace, radius and volume of the robot
    #plotWorkspace(UR3E(),90)  
   
 #Uncomment to see the workspace extreme x and y
    #WorkSpaceReachRadius(UR3E())

 #Uncomment to find a joint state for a given end effector pose
    #giveEEPoseAndDetermineJointStateAndEvidence()

 #Uncomment to read the bag file and replicate Robot movement
    #read_bag('/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/Assignment_1/2018-03-20-18-34-46.bag')
    #move_from_text('/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/bag_q.txt') 