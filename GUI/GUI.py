import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import roboticstoolbox as rtb #robotics toolbox used for various calculations
from roboticstoolbox import *
from ir_support import DHRobot3D #Industrial Robotics Specific
import spatialgeometry as geometry
from spatialmath import *
import numpy as np
from GUI import Controller


class GUI(tk.Frame):
    def __init__(self, Name, UR3, BB):
        self.control = Controller.XboxController()
        self.root = tk.Tk()
        #Main Window
        tk.Frame.__init__(self, self.root)
        self.grid()
        self.CreateWidgets()
        # self.Update()
        self.root.title(Name)
        self.root.resizable(False, False)
        #Variables
        self.UR3Transform = SE3(0,0,0)
        self.BBTransform = SE3(0,0,0)
        #Variables
        self.UR3 = UR3
        self.BB = BB

    def Refresh(self):
        # self.root.after(100, self.Refresh())
        self.ControllerJog()
        self.update()

    def CreateWidgets(self):
        self.xPosButton = ttk.Button(self.root, text="+X", command=lambda : self.Jog('+x'))
        self.yPosButton = ttk.Button(self.root, text="+Y", command=lambda : self.Jog('+y'))
        self.yNegButton = ttk.Button(self.root, text="-Y", command=lambda : self.Jog('-y'))
        self.zPosButton = ttk.Button(self.root, text="+Z", command=lambda : self.Jog('+z'))
        self.zNegButton = ttk.Button(self.root, text="-Z", command=lambda : self.Jog('-z'))
        self.xNegButton = ttk.Button(self.root, text="-X", command=lambda : self.Jog('-x'))
        self.xPosButton.grid(column=1,row=0)
        self.xNegButton.grid(column=1,row=2)
        self.yPosButton.grid(column=0,row=1)
        self.yNegButton.grid(column=2,row=1)
        self.zPosButton.grid(column=3,row=0)
        self.zNegButton.grid(column=3,row=1)

    def Jog(self, dimension): #Jogging the robot with the Tkinter GUI
        match dimension:
            case '+x':
                self.BBTransform.x += 0.1
            case '-x':
                self.BBTransform.x -= 0.1
            case '+y':
                self.BBTransform.y += 0.1
            case '-y':
                self.BBTransform.y -= 0.1
            case '+z':
                self.BBTransform.z += 0.1
            case '-z':
                self.BBTransform.z -= 0.1


    def ControllerJog(self): #Jogs the robot with the Xbox controller
            self.BBTransform.x += Controller.XboxController.read(self.control)[0]/10 #X-Axis control
            self.BBTransform.z += Controller.XboxController.read(self.control)[1]/10 #Z-Axis control
            self.BBTransform.y += Controller.XboxController.read(self.control)[3]/10 #Positive Y-Axis
            self.BBTransform.y -= Controller.XboxController.read(self.control)[2]/10 #Negative Y-Axis
            if Controller.XboxController.read(self.control)[6] or Controller.XboxController.read(self.control)[7] == 1: #E-Stop
                exit()