import roboticstoolbox as rtb #robotics toolbox used for various calculations
from roboticstoolbox import *
from ir_support import DHRobot3D #Industrial Robotics Specific
import spatialgeometry as geometry
from spatialmath import *
import numpy as np
import swift
#Custom Imports
from GUI import Controller

class GUI():
    def __init__(self, env, UR3, Itz):
        #Variables
        self.UR3 = UR3
        self.Itz = Itz
        self.ActiveBot = self.Itz
        self.TargetEEPos = SE3(0.1,0.1,0.1)
        self.control = Controller.XboxController()
        #Main Window
        self.CreateWidgets(env)

    def Refresh(self, env):
        self.ControllerJog(env)
        self.ActiveBot.q = self.ActiveBot.ikine_LM(self.TargetEEPos, q0=self.ActiveBot.q, joint_limits=True).q #A little Glitchy need to figure out better IK
        self.EEPosx.desc = ("X: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).x, 2))) 
        self.EEPosy.desc = ("Y: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).y, 2)))
        self.EEPosz.desc = ("Z: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).z, 2)))
        self.ActiveBotLabel.desc = ("Active Robot: " + str(self.ActiveBot.name))

    def set_joint(self, j, value): #Sets the joint angle
        self.ActiveBot.q[j] = np.deg2rad(float(value))

    def ChangeBot(self): #Changes which robot is currently being controlled
        if self.ActiveBot == self.Itz:
            self.ActiveBot = self.UR3
        elif self.ActiveBot == self.UR3:
            self.ActiveBot = self.Itz

    def CreateWidgets(self, env): #Add buttons, text and sliders to the swift environment
        self.ActiveBotLabel = swift.Label("Active Robot: " + str(self.ActiveBot.name))
        env.add(swift.Button(lambda x : self.ChangeBot(), 'Change Bot'))
        self.EEPosx = swift.Label("X: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).x, 2))) 
        self.EEPosy = swift.Label("Y: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).y, 2)))
        self.EEPosz = swift.Label("Z: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).z, 2)))
        env.add(self.ActiveBotLabel)
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
        for l in self.Itz.links: #Fix prismatic joints (both robots)
            if l.isjoint:
                env.add(swift.Slider(lambda x, j=j: self.set_joint(j, x),
                                        min=np.round(np.rad2deg(l.qlim[0]), 2),
                                        max=np.round(np.rad2deg(l.qlim[1]), 2),
                                        step=1,
                                        value=np.round(np.rad2deg(self.Itz.q[j]), 2),
                                        desc="Itzamna Joint " + str(j),
                                        unit="&#176;",))
                j += 1

    def Jog(self, dimension): #Jogging the robot with the Tkinter GUI
        match dimension:
            case '+x':
                self.TargetEEPos.x += 0.1 
            case '-x':
                self.TargetEEPos.x -= 0.1
            case '+y':
                self.TargetEEPos.y += 0.1
            case '-y':
                self.TargetEEPos.y -= 0.1
            case '+z':
                self.TargetEEPos.z += 0.1
            case '-z':
                self.TargetEEPos.z -= 0.1


    def ControllerJog(self, env): #Jogs the robot with the Xbox controller
            self.TargetEEPos.x += Controller.XboxController.read(self.control)[0]/10 #X-Axis control
            self.TargetEEPos.z += Controller.XboxController.read(self.control)[1]/10 #Z-Axis control
            self.TargetEEPos.y += Controller.XboxController.read(self.control)[3]/10 #Positive Y-Axis
            self.TargetEEPos.y -= Controller.XboxController.read(self.control)[2]/10 #Negative Y-Axis
            if self.TargetEEPos.x > 2.5:
                self.TargetEEPos.x = 2.5
            if self.TargetEEPos.z > 2.5:
                self.TargetEEPos.z = 2.5
            if self.TargetEEPos.y > 2:
                self.TargetEEPos.y = 2
            if self.TargetEEPos.x < 0:
                self.TargetEEPos.x = 0
            if self.TargetEEPos.z < 0:
                self.TargetEEPos.z = 0
            if self.TargetEEPos.y < 0:
                self.TargetEEPos.y = 0
            if Controller.XboxController.read(self.control)[8] or Controller.XboxController.read(self.control)[9] == 1: #E-Stop
                exit()
            if Controller.XboxController.read(self.control)[4]:
                pass
            if Controller.XboxController.read(self.control)[5]:
                pass