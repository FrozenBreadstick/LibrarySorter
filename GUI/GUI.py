import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import roboticstoolbox as rtb #robotics toolbox used for various calculations
from roboticstoolbox import *
from ir_support import DHRobot3D #Industrial Robotics Specific

class GUI():
    def __init__(self, UR3, BB):
        self.root = tk.Tk()
        self.root.title("GUI")
        self.UR3 = UR3
        self.BB = BB
        #Variables
        EEx = str(UR3.fkine(UR3.q).x)
        EEy = str(UR3.fkine(UR3.q).y)
        EEz = str(UR3.fkine(UR3.q).z)
        self.x_label = ttk.Label(self.root, text="X:") #Create an X input field
        self.x_label.grid(row=1, column=0)
        self.x_label = ttk.Label(self.root, text=EEx) #Create an X input field
        self.x_label.grid(row=1, column=1)
        self.x_entry = ttk.Scale(self.root)
        self.x_entry.grid(row=1, column=2)

        self.x_label = ttk.Label(self.root, text="Y:") #Create an X input field
        self.x_label.grid(row=2, column=0)
        self.x_label = ttk.Label(self.root, text=EEy) #Create an X input field
        self.x_label.grid(row=2, column=1)
        self.x_entry = ttk.Scale(self.root)
        self.x_entry.grid(row=2, column=2)

        self.x_label = ttk.Label(self.root, text="Z:") #Create an X input field
        self.x_label.grid(row=3, column=0)
        self.x_label = ttk.Label(self.root, text=EEz) #Create an X input field
        self.x_label.grid(row=3, column=1)
        self.x_entry = ttk.Scale(self.root)
        self.x_entry.grid(row=3, column=2)

        

        self.pos_label = ttk.Label(self.root, text="End Effector: (0.0, 0.0, 0.0)") #Create a label that show the current end effector translation position
        self.pos_label.grid(row=4, column=0, columnspan=3)
