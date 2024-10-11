import tkinter as tk
from tkinter import ttk
from tkinter import messagebox

class GUI():
    def __init__(self, UR3, BB):
        self.root = tk.Tk()
        self.root.title("GUI")
        self.UR3 = UR3
        self.BB = BB

        self.x_label = ttk.Label(self.root, text="X (0.579,-1.3782):") #Create an X input field
        self.x_label.grid(row=0, column=0, padx=10, pady=10)
        self.x_entry = ttk.Scale(self.root)
        self.x_entry.grid(row=0, column=1, padx=10, pady=10)

        self.label_y = ttk.Label(self.root, text="Y (0.5789,-0.5416):") #Create a Y input field
        self.label_y.grid(row=1, column=0, padx=10, pady=10)
        self.y_entry = ttk.Entry(self.root)
        self.y_entry.grid(row=1, column=1, padx=10, pady=10)

        self.z_label = ttk.Label(self.root, text="Z (0.6935,-0.3897):") #Create a Z input field
        self.z_label.grid(row=2, column=0, padx=10, pady=10)
        self.z_entry = ttk.Entry(self.root)
        self.z_entry.grid(row=2, column=1, padx=10, pady=10)

        self.pos_label = ttk.Label(self.root, text="End Effector: (0.0, 0.0, 0.0)") #Create a label that show the current end effector translation position
        self.pos_label.grid(row=3, column=0, columnspan=2, pady=10)
