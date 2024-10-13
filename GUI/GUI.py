import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import roboticstoolbox as rtb #robotics toolbox used for various calculations
from roboticstoolbox import *
from ir_support import DHRobot3D #Industrial Robotics Specific



class GUI():
    #Main Window
    root = tk.Tk()
    root.geometry("600x450")
    root.resizable(False, False)
    #Variables
    x = 0
    y = 0
    z = 0
    

    def __init__(self,Name, UR3, BB):
        #Variables
        self.root.title(Name)
        self.UR3 = UR3
        self.BB = BB
        xPosButton = ttk.Button(GUI.root, text="+X", command=lambda : GUI.Jog('+x'))
        xNegButton = ttk.Button(GUI.root, text="-X", command=lambda : GUI.Jog('-x'))
        yPosButton = ttk.Button(GUI.root, text="+Y", command=lambda : GUI.Jog('+y'))
        yNegButton = ttk.Button(GUI.root, text="-Y", command=lambda : GUI.Jog('-y'))
        zPosButton = ttk.Button(GUI.root, text="+Z", command=lambda : GUI.Jog('+z'))
        zNegButton = ttk.Button(GUI.root, text="-Z", command=lambda : GUI.Jog('-z'))
        xPosButton.grid(column=1,row=0)
        xNegButton.grid(column=1,row=2)
        yPosButton.grid(column=0,row=1)
        yNegButton.grid(column=2,row=1)
        zPosButton.grid(column=3,row=0)
        zNegButton.grid(column=3,row=1)

    def Update(self):
        EEx = str(self.UR3.fkine(self.UR3.q).x)
        EEy = str(self.UR3.fkine(self.UR3.q).y)
        EEz = str(self.UR3.fkine(self.UR3.q).z)

        GUI.root.mainloop()

    def Jog(dimension):
        match dimension:
            case '+x':
                GUI.x += 1
            case '-x':
                GUI.x -= 1
            case '+y':
                GUI.y += 1
            case '-y':
                GUI.y -= 1
            case '+z':
                GUI.z += 1
            case '-z':
                GUI.z -= 1

        print(GUI.x, " ", GUI.y, " ", GUI.z)