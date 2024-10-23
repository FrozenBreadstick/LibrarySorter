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

        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.wx = 0
        self.wy = 0
        self.wz = 0
        self.dx = np.array([self.vx, self.vy, self.vz, self.wx ,self.wy, self.wz])

        self.Sliders = {}

        self.Control = Controller.XboxController()
        #Main Window
        self.CreateWidgets(env)

    def Refresh(self, env):
        self.ControllerJog(env)
        self.JointUpdate()
        self.WidgetUpdate()

    def JointUpdate(self):
        _lambda = 0.1
        J = self.ActiveBot.jacob0(self.ActiveBot.q)
        JinvDLS = np.linalg.inv((J.T @ J) + _lambda**2 * np.eye(7)) @ J.T
        dq = JinvDLS @ self.dx
        self.ActiveBot.q = self.ActiveBot.q + dq
    
    def WidgetUpdate(self):
        self.ActiveBotLabel.desc = ("Active Robot: " + str(self.ActiveBot.name))
        #Positions
        self.EEPosx.desc = ("X: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).x, 2))) 
        self.EEPosy.desc = ("Y: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).y, 2)))
        self.EEPosz.desc = ("Z: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).z, 2)))
        #Sliders
        j = 0
        for l in self.Sliders: #Change Slider Limits
            self.Sliders["Link{0}".format(j+1)].cb = lambda x, j=j: self.set_joint(j, x)
            self.Sliders["Link{0}".format(j+1)].value = np.round(np.rad2deg(self.ActiveBot.q[j]), 2)
            j += 1

    def set_joint(self, j, value): #Sets the joint angle
        # self.ActiveBot.q[j] = np.deg2rad(float(value)) #When updating value on slider this gets run causing jittering
        print(value)

    def ChangeBot(self): #Changes which robot is currently being controlled
        if self.ActiveBot == self.Itz:
            self.ActiveBot = self.UR3
        elif self.ActiveBot == self.UR3:
            self.ActiveBot = self.Itz
        j = 0
        for l in self.ActiveBot.links: #Change Slider Limits
            if l.isjoint:
                self.Sliders["Link{0}".format(j+1)].min = np.round(np.rad2deg(l.qlim[0]), 2)
                self.Sliders["Link{0}".format(j+1)].max = np.round(np.rad2deg(l.qlim[1]), 2)
                j += 1

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
        env.add(swift.Button(lambda x : self.Jog('+x'), '+X')) #Might Remove buttons
        env.add(swift.Button(lambda x : self.Jog('-x'), '-X'))
        env.add(swift.Button(lambda x : self.Jog('+y'), '+Y'))
        env.add(swift.Button(lambda x : self.Jog('-y'), '-Y'))
        env.add(swift.Button(lambda x : self.Jog('+z'), '+Z'))
        env.add(swift.Button(lambda x : self.Jog('-z'), '-Z'))
        j = 0
        for l in self.ActiveBot.links: #Fix prismatic joints (both robots)
            if l.isjoint:
                self.Sliders["Link{0}".format(str(j+1))] = (swift.Slider(lambda x, j=j: self.set_joint(j, x),
                                                            min=np.round(np.rad2deg(l.qlim[0]), 2),
                                                            max=np.round(np.rad2deg(l.qlim[1]), 2),
                                                            step=1,
                                                            value=np.round(np.rad2deg(self.Itz.q[j]), 2),
                                                            desc="Joint " + str(j),
                                                            unit="&#176;",))
                env.add(self.Sliders["Link{0}".format(str(j+1))])
                j += 1

    def Jog(self, dimension): #Jogging the robot with the Tkinter GUI
        match dimension:
            case '+x':
                self.vx += 0.1 
            case '-x':
                self.vx -= 0.1
            case '+y':
                self.vy += 0.1
            case '-y':
                self.vy -= 0.1
            case '+z':
                self.vz += 0.1
            case '-z':
                self.vz -= 0.1

    def ControllerJog(self, env): #Jogs the robot with the Xbox controller
        kv = 0.3
        self.vx = kv * Controller.XboxController.read(self.Control)[0]/10 #X-Axis control
        self.vz = kv * Controller.XboxController.read(self.Control)[1]/10 #Z-Axis control
        self.vy = kv * (Controller.XboxController.read(self.Control)[3]/10 - Controller.XboxController.read(self.Control)[2]/10) #Y-Axis Control
        self.dx = np.array([self.vx, self.vy, self.vz, self.wx ,self.wy, self.wz])
        if Controller.XboxController.read(self.Control)[8] or Controller.XboxController.read(self.Control)[9] == 1: #E-Stop
            exit()
        if Controller.XboxController.read(self.Control)[4]:
            pass
        if Controller.XboxController.read(self.Control)[5]:
            pass