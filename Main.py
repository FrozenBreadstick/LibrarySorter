#Imports
from ir_support import DHRobot3D #Industrial Robotics Specific
import swift #Rendering for the robots in browser
from spatialmath import * #matrix math
from spatialmath.base import *
import spatialgeometry as geometry
import roboticstoolbox as rtb #robotics toolbox used for various calculations
from roboticstoolbox import *
import numpy as np #Additional math operations
import os
import time
import logging
import sys
#custom other files
from UR3E import UR3E
from GUI import GUI

UR3 = UR3E.UR3E()
BB = 2 #Temp place holder

env = swift.Swift()
env.launch(realtime=True)
env.set_camera_pose([1,-2,1], [1,0,1])

temp = geometry.Mesh('Assessment_1\Brick.dae')
env.add(temp)

ControlPanel = GUI.GUI("Control Panel",UR3,temp, env)

def main():
    temp.T = ControlPanel.BBTransform
    env.step()
    pass

if __name__ == "__main__":
    while True:
        main()
        ControlPanel.Refresh()