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
from Models import Itzamna

env = swift.Swift()
env.launch(realtime=True)

UR3 = UR3E.UR3E()
Itz = Itzamna.Itzamna()

UR3.add_to_env(env)
Itz.add_to_env(env)

# temp = geometry.Mesh('Assessment_1\Brick.dae')
# env.add(temp)

ControlPanel = GUI.GUI(env, UR3, Itz)
def main():
    Itz.q = ControlPanel.Itz.q
    UR3.q = ControlPanel.UR3.q
    env.step()
    pass

if __name__ == "__main__":
    env.set_camera_pose([1.3,-2.3,1.3], [1.3,0,1.3])
    while True:
        main()
        ControlPanel.Refresh(env)