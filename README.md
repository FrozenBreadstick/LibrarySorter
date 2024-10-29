# LibrarySorter

## Synopsis

This project is a robotic simulation using *[Peter Corkes Robotics-Toolbox-Python](https://github.com/petercorke/robotics-toolbox-python "Robotics Toolbox")* for the **UTS Subject 41013 Industrial Robotics**

This project is designed to sort library books based on size and colour, using the UR3 and another custom robot

## The Robots

### The UR3

The UR3 is used to do the initial sorting of the books into piles so Itzamna can grab them.
![image of the LinearUR3](/Docs/Figs/LinearUR3.png "LinearUR3")

### Itzamna

Itzamna Takes the books from the UR3 and sorts them into the shelf
It is a 7DOF robot with 2 prismatic joints to move it around the shelf, similar to a CNC Router.
![image of Itzamna](/Docs/Figs/Itzamna.png "Itzamna")

## Installation

This project uses some external libraries to aid in it's function. These include:

### BagPy

This is used to read ROS .bag files: *[BagPy Documentation](https://pypi.org/project/bagpy/)*

```shell script
pip install bagpy
```

### Inputs

This is used to read the inputs from the Xbox Controller: *[Inputs Documentation](https://pypi.org/project/inputs/)*

```shell script
pip install inputs
```

### PySerial

This is used to read the inputs from the Arduino: *[PySerial Documentation](https://pypi.org/project/pyserial/)*

```shell script
pip install pyserial
```
