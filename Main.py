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
BB = 2
test = GUI.GUI(UR3,BB)

def main():
    pass

if __name__ == "__main__":
    main()
    test.root.mainloop()
    