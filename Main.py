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
#custom other files
from UR3E import UR3E
from GUI import GUI
from Models import Itzamna

env = swift.Swift()
env.launch(realtime=True)

UR3 = UR3E.UR3E()
Itz = Itzamna.Itzamna()

UR3.add_to_env(env)
Itz.add_to_env(env)

ControlPanel = GUI.GUI(env, UR3, Itz)

t = geometry.Mesh('Assessment_1\Brick.dae', [1,1,1], collision=True, pose=SE3(0.3,0.3,0.3))
tc = RectangularPrism(0.5,0.5,0.5, center=[0.8,0.8,0.8])
vertex, faces, face_normals = tc.get_data()

env.add(tc)
def main():
    Itz.q = ControlPanel.Itz.q
    UR3.q = ControlPanel.UR3.q
    print(is_collision(Itz, Itz.q, faces, vertex, face_normals))
    env.step()
    pass



#-------------------Collision functions from labs-------------------#
"""
These Functions were used in Lab 5, they should work, will test and trim 'fat'
They need a collision box made with the Rectangular prism above, 
so we need to put one of them around all the possible collision areas
"""
#Individual q Collision detection
def is_collision(robot, q, faces, vertex, face_normals, return_once_found = True, fig = None):
    """
    This is based upon the output of questions 2.5 and 2.6
    Given a robot model (robot), and trajectory (i.e. joint state vector) (q_matrix)
    and triangle obstacles in the environment (faces,vertex,face_normals)
    """
    result = False
    if fig is not None:
        ax = fig.ax
        # Get the transform of every joint (i.e. start and end of every link)
    tr = get_link_poses(robot, q)
    
    # Go through each link and also each triangle face
    for i in range(np.size(tr,2)-1):
        for j, face in enumerate(faces):
            vert_on_plane = vertex[face][0]
            intersect_p, check = line_plane_intersection(face_normals[j], 
                                                        vert_on_plane, 
                                                        tr[i][:3,3], 
                                                        tr[i+1][:3,3])
            # list of all triangle combination in a face
            triangle_list  = np.array(list(combinations(face,3)),dtype= int)
            if check == 1:
                for triangle in triangle_list:
                    if is_intersection_point_inside_triangle(intersect_p, vertex[triangle]):
                        if fig is not None:
                            ax.plot(intersect_p[0], intersect_p[1], intersect_p[2], 'g*')
                        # print('Intersection')
                        result = True
                        if return_once_found:
                            return result
                        break
    return result
#ORIGINAL VERSION - Meant for full trajectories
# def is_collision(robot, q_matrix, faces, vertex, face_normals, return_once_found = True, fig = None):
#     """
#     This is based upon the output of questions 2.5 and 2.6
#     Given a robot model (robot), and trajectory (i.e. joint state vector) (q_matrix)
#     and triangle obstacles in the environment (faces,vertex,face_normals)
#     """
#     result = False
#     if fig is not None:
#         ax = fig.ax
#     for i, q in enumerate(q_matrix):
#         # Get the transform of every joint (i.e. start and end of every link)
#         tr = get_link_poses(robot, q)
        
#         # Go through each link and also each triangle face
#         for i in range(np.size(tr,2)-1):
#             for j, face in enumerate(faces):
#                 vert_on_plane = vertex[face][0]
#                 intersect_p, check = line_plane_intersection(face_normals[j], 
#                                                             vert_on_plane, 
#                                                             tr[i][:3,3], 
#                                                             tr[i+1][:3,3])
#                 # list of all triangle combination in a face
#                 triangle_list  = np.array(list(combinations(face,3)),dtype= int)
#                 if check == 1:
#                     for triangle in triangle_list:
#                         if is_intersection_point_inside_triangle(intersect_p, vertex[triangle]):
#                             if fig is not None:
#                                 ax.plot(intersect_p[0], intersect_p[1], intersect_p[2], 'g*')
#                             # print('Intersection')
#                             result = True
#                             if return_once_found:
#                                 return result
#                             break
#     return result

def get_link_poses(robot:DHRobot,q=None):
    """
    :param q robot joint angles
    :param robot -  seriallink robot model
    :param transforms - list of transforms
    """
    if q is None:
        return robot.fkine_all().A
    return robot.fkine_all(q).A

def is_intersection_point_inside_triangle(intersect_p, triangle_verts):
    u = triangle_verts[1, :] - triangle_verts[0, :]
    v = triangle_verts[2, :] - triangle_verts[0, :]

    uu = np.dot(u, u)
    uv = np.dot(u, v)
    vv = np.dot(v, v)

    w = intersect_p - triangle_verts[0, :]
    wu = np.dot(w, u)
    wv = np.dot(w, v)

    D = uv * uv - uu * vv

    # Get and test parametric coords (s and t)
    s = (uv * wv - vv * wu) / D
    if s < 0.0 or s > 1.0:  # intersect_p is outside Triangle
        return 0

    t = (uv * wu - uu * wv) / D
    if t < 0.0 or (s + t) > 1.0:  # intersect_p is outside Triangle
        return False

    return True  # intersect_p is in Triangle

#-----------------------------------Main----------------------------#
import threading
from libBot_Assets import libBot #Import the 3D model of the robot

def EnvironmentSetup(): 
     rLibot = DHRobot3D.LibraryBot()

def mainCode():
    
  
    
    pass
    
    
def check_Stop_press():
    #This function will check for a key press
    #Connect to read pin off arduino
    pass

def run():
    #This part do threads to check for a key press
    t1 = threading.Thread(target=check_Stop_press)
    t2=threading.Thread(target=mainCode)
    


if __name__ == "__main__":
    env.set_camera_pose([1.3,-2.3,1.3], [1.3,0,1.3])
    while True:
        run()
        ControlPanel.Refresh(env)