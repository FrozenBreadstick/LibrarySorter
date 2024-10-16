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
import swift

class GUI(tk.Frame):
    def __init__(self, Name, UR3, BB, env):
        #Variables
        # self.root = tk.Tk()
        self.UR3 = UR3
        self.BB = BB
        self.control = Controller.XboxController()
        self.UR3Transform = SE3(0,0,0)
        self.BBTransform = SE3(0,0,0)
        # self.CamPos = [1.5, -2, 1.5]
        #Main Window
        # tk.Frame.__init__(self, self.root)
        # self.grid()
        self.CreateWidgets(env)
        # self.root.title(Name)
        # self.root.resizable(False, False)

    def Refresh(self, env):
        self.ControllerJog(env)
        self.EEPosx.desc = ("X: " + str(self.BBTransform.x))
        self.EEPosy.desc = ("Y: " + str(self.BBTransform.y))
        self.EEPosz.desc = ("Z: " + str(self.BBTransform.z))
        # self.update()

    def set_joint(self, j, value):
        self.BB.q[j] = np.deg2rad(float(value))

    def CreateWidgets(self, env):
        # self.xPosButton = ttk.Button(self.root, text="+X", command=lambda : self.Jog('+x'))
        # self.yPosButton = ttk.Button(self.root, text="+Y", command=lambda : self.Jog('+y'))
        # self.yNegButton = ttk.Button(self.root, text="-Y", command=lambda : self.Jog('-y'))
        # self.zPosButton = ttk.Button(self.root, text="+Z", command=lambda : self.Jog('+z'))
        # self.zNegButton = ttk.Button(self.root, text="-Z", command=lambda : self.Jog('-z'))
        # self.xNegButton = ttk.Button(self.root, text="-X", command=lambda : self.Jog('-x'))
        # self.ModeLabel = ttk.Label(self.root, text='')
        # self.ModeLabel.grid(column=1, row=3)
        # self.xPosButton.grid(column=1,row=0)
        # self.xNegButton.grid(column=1,row=2)
        # self.yPosButton.grid(column=0,row=1)
        # self.yNegButton.grid(column=2,row=1)
        # self.zPosButton.grid(column=3,row=0)
        # self.zNegButton.grid(column=3,row=1)
        self.EEPosx = swift.Label("X: " + str(self.BBTransform.x))
        self.EEPosy = swift.Label("Y: " + str(self.BBTransform.y))
        self.EEPosz = swift.Label("Z: " + str(self.BBTransform.z))
        env.add(self.EEPosx)
        env.add(self.EEPosy)
        env.add(self.EEPosz)
        env.add(swift.Button(lambda x : self.Jog('+x'), '+X'))
        env.add(swift.Button(lambda x : self.Jog('-x'), '-X'))
        env.add(swift.Button(lambda x : self.Jog('+y'), '+Y'))
        env.add(swift.Button(lambda x : self.Jog('-y'), '-Y'))
        env.add(swift.Button(lambda x : self.Jog('+z'), '+Z'))
        env.add(swift.Button(lambda x : self.Jog('-z'), '-Z'))
        j = 0
        for l in self.BB.links:
            if l.isjoint:
                env.add(swift.Slider(lambda x, j=j: self.set_joint(j, x),
                                        min=np.round(np.rad2deg(l.qlim[0]), 2),
                                        max=np.round(np.rad2deg(l.qlim[1]), 2),
                                        step=1,
                                        value=np.round(np.rad2deg(self.BB.q[j]), 2),
                                        desc="Itzamna Joint " + str(j),
                                        unit="&#176;",))
                j += 1
                print(j, ' ', l)

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


    def ControllerJog(self, env): #Jogs the robot with the Xbox controller
            self.BBTransform.x += Controller.XboxController.read(self.control)[0]/10 #X-Axis control
            self.BBTransform.z += Controller.XboxController.read(self.control)[1]/10 #Z-Axis control
            self.BBTransform.y += Controller.XboxController.read(self.control)[3]/10 #Positive Y-Axis
            self.BBTransform.y -= Controller.XboxController.read(self.control)[2]/10 #Negative Y-Axis
            if self.BBTransform.x > 2.5:
                self.BBTransform.x = 2.5
            if self.BBTransform.z > 2.5:
                self.BBTransform.z = 2.5
            if self.BBTransform.y > 2:
                self.BBTransform.y = 2
            if self.BBTransform.x < 0:
                self.BBTransform.x = 0
            if self.BBTransform.z < 0:
                self.BBTransform.z = 0
            if self.BBTransform.y < 0:
                self.BBTransform.y = 0
            # self.CamPos[0] += Controller.XboxController.read(self.control)[6]/10
            # self.CamPos[2] += Controller.XboxController.read(self.control)[7]/10
            if Controller.XboxController.read(self.control)[8] or Controller.XboxController.read(self.control)[9] == 1: #E-Stop
                exit()
            if Controller.XboxController.read(self.control)[4]:
                pass
            if Controller.XboxController.read(self.control)[5]:
                pass