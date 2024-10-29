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
        #Controller Joint smoothing variables
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.wx = 0
        self.wy = 0
        self.wz = 0
        self.dx = np.array([self.vx, self.vy, self.vz, self.wx ,self.wy, self.wz])
        #Environment Variables
        self.Sliders = {}
        self.ControllerMode = False
        self.Stopped = False
        #Gripper
        self.GripperState = False
        #Controller Variable
        try:
            self.Control = Controller.XboxController()
        except Exception:
            pass
        #Main Window
        self.CreateWidgets(env)

    def Refresh(self): #Updates the state if the GUI
        self.ControllerJog()
        self.JointUpdate()
        self.WidgetUpdate()

    def JointUpdate(self): #Changes q values when using the controller
        if self.ControllerMode == True:
            _lambda = 0.1
            J = self.ActiveBot.jacob0(self.ActiveBot.q)
            JinvDLS = np.linalg.inv((J.T @ J) + _lambda**2 * np.eye(7)) @ J.T
            dq = JinvDLS @ self.dx
            self.ActiveBot.q = self.ActiveBot.q + dq
        i = 0
        for l in self.ActiveBot.links: #keeps qlims when using controller
            if self.ActiveBot.q[i] <= l.qlim[0]:
                self.ActiveBot.q[i] = l.qlim[0]
            if self.ActiveBot.q[i] >= l.qlim[1]:
                self.ActiveBot.q[i] = l.qlim[1]
            i += 1


    def WidgetUpdate(self): #updates Widget descriptions and values
        self.ChangeBotButton.desc = (str(self.ActiveBot.name))
        self.ControllerButton.desc = ('Controller Mode: ' + str(self.ControllerMode))
        #Positions
        self.EEPosx.desc = ("X: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).x, 2))) 
        self.EEPosy.desc = ("Y: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).y, 2)))
        self.EEPosz.desc = ("Z: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).z, 2)))
        #Sliders
        j = 0
        if self.ControllerMode == False:
            for l in self.Sliders: #Change Slider Limits
                if self.Sliders["Link{0}".format(j+1)].value != np.round(np.rad2deg(self.ActiveBot.q[j]), 2):
                    self.Sliders["Link{0}".format(j+1)].value = np.round(np.rad2deg(self.ActiveBot.q[j]), 2)
                j += 1
                
    def set_joint(self, j, value): #Sets the joint angle
        self.ActiveBot.q[j] = np.deg2rad(float(value)) #When updating value on slider this gets run causing jittering

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
        env.add(swift.Button(lambda x : self.ESTOP(), 'E-Stop'))
        self.ControllerButton = swift.Button(lambda x : self.ModeChange(), 'Controller Mode: ' + str(self.ControllerMode))
        self.ChangeBotButton = swift.Button(lambda x : self.ChangeBot(), str(self.ActiveBot.name))
        self.EEPosx = swift.Label("X: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).x, 2))) 
        self.EEPosy = swift.Label("Y: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).y, 2)))
        self.EEPosz = swift.Label("Z: " + str(np.round(self.ActiveBot.fkine(self.ActiveBot.q).z, 2)))
        env.add(self.ControllerButton)
        env.add(self.ChangeBotButton)
        env.add(self.EEPosx)
        env.add(self.EEPosy)
        env.add(self.EEPosz)
        j = 0
        for l in self.ActiveBot.links: #Adds Sliders for teach functionality
            if l.isjoint:
                self.Sliders["Link{0}".format(str(j+1))] = (swift.Slider(lambda x, j=j: self.set_joint(j, x),
                                                            min=np.round(np.rad2deg(l.qlim[0]), 2),
                                                            max=np.round(np.rad2deg(l.qlim[1]), 2),
                                                            step=1,
                                                            value=np.round(np.rad2deg(self.ActiveBot.q[j]), 2),
                                                            desc="Joint " + str(j),
                                                            unit="&#176;"))
                env.add(self.Sliders["Link{0}".format(str(j+1))])
                j += 1
        env.add(swift.Button(lambda x : exit(), "Exit Simulation"))

    def ESTOP(self):
        self.Stopped = not self.Stopped

    def ModeChange(self):
        self.ControllerMode = not self.ControllerMode

    def ControllerJog(self): #Jogs the robot with the Xbox controller
        kv = 0.7
        # kw = 0.8
        self.vx = kv * Controller.XboxController.read(self.Control)[0]/10 #X-Axis control
        self.vz = kv * Controller.XboxController.read(self.Control)[1]/10 #Z-Axis control
        self.vy = kv * (Controller.XboxController.read(self.Control)[3]/10 - Controller.XboxController.read(self.Control)[2]/10) #Y-Axis Control
        # self.ActiveBot.q[len(self.ActiveBot.q)-1] = np.arctan2(Controller.XboxController.read(self.Control)[7]/10, Controller.XboxController.read(self.Control)[6]/10)
        self.dx = np.array([self.vx, self.vy, self.vz, self.wx ,self.wy, self.wz])
        if Controller.XboxController.read(self.Control)[8] or Controller.XboxController.read(self.Control)[9] == 1: #E-Stop
            self.ESTOP()
        if Controller.XboxController.read(self.Control)[4]: #Activate Gripper
            self.GripperState = True
        if Controller.XboxController.read(self.Control)[5]: #Deactivate Gripper
            self.GripperState = False