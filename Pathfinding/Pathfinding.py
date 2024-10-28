import numpy as np
import heapq #Heapq is a list-like alternative that organizes items based on a score, allows for quick and efficient access to smallest and highest priority items. Keeps items stored in order
import random
from concurrent.futures import ThreadPoolExecutor, as_completed
import matplotlib.pyplot as plt
from roboticstoolbox import *
import roboticstoolbox as rtb
from ir_support.robots.DHRobot3D import DHRobot3D
from spatialmath import SE3
import logging
from math import *
logging.basicConfig( level=logging.INFO, format='%(asctime)s - %(message)s', handlers=[logging.FileHandler("execution_log.log"), logging.StreamHandler()])

class Node: #Class for storing node position and cost during calculations
    def __init__(self, position, parent=None, direction=0):
        self.position = position #Store position
        self.parent = parent #Store preceeding node
        self.direction = direction
        self.g = float('inf') #Cost to reach this node from start node
        self.h = 0 #Cost "as the crow flies" to the end goal
        self.f = float('inf') #Total cost (g+h)

    def update_costs(self, g, h): #Updates the costs
        self.g = g
        self.h = h
        self.f = g + h

    def __lt__(self, other): #Property that allows the cost of nodes to be compared directly
        return self.f < other.f
    
class ItzThetaStarPathing:
    def __init__(self, robot: DHRobot3D, shapes: list = None): 
        self.robot = robot
        self.shapes = shapes #Shapes should be a list of all scene objects you wish to take into account during calculations

    def update_shapes(self, shapes: list):
        self.shapes = shapes

    def collision_check(self, p1, p2):
        if self.shapes == None:
            return False
        p = SE3(p2[0], p2[1], p2[2])
        pose = self.robot.ikine_LM(Tep = p, q0 = self.robot.q, joint_limits = True)
        for shape in self.shapes:
            if self.robot.iscollided(shape, pose):
                return True
        return False

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b)) #Simple calculation, distance between two nodes

    def get_neighbours(self, node, step_size=0.5): #Get's all possible neighbour positions around a node
        directions = [
            (step_size, 0, 0), (-step_size, 0, 0),
            (0, step_size, 0), (0, -step_size, 0),
            (0, 0, step_size), (0, 0, -step_size),
            (step_size, step_size, 0), (step_size, -step_size, 0),
            (step_size, 0, step_size), (step_size, 0, -step_size),
            (-step_size, step_size, 0), (-step_size, -step_size, 0),
            (-step_size, 0, step_size), (-step_size, 0, -step_size),
            (0, step_size, step_size), (0, step_size, -step_size),
            (0, -step_size, step_size), (0, -step_size, -step_size),
            (step_size, step_size, step_size), (step_size, step_size, -step_size),
            (step_size, -step_size, step_size), (step_size, -step_size, -step_size),
            (-step_size, step_size, step_size), (-step_size, step_size, -step_size),
            (-step_size, -step_size, step_size), (-step_size, -step_size, -step_size),
        ]
        
        neighbours = []
        for dx, dy, dz in directions: #create a list of all neighbours
            neighbour_pos = (node.position[0] + dx, node.position[1] + dy, node.position[2] + dz)
            neighbours.append(neighbour_pos)
        return neighbours

    def theta_star(self, goal, start = None, max_threads=4, step_size=1): #Method for the Theta# implementation
        if start == None: #Ensure a value of start for calculations
            start = self.robot.fkine(self.robot.q).t
            start = (round(start[0],2), round(start[1],2), round(start[2],2),)
        start = tuple(i*20 for i in start) #Work in a factor of ten
        goal = tuple(i*20 for i in goal) #Work in a factor of ten
        k = 0
        tol = 0.6
        open_set = [] #Nodes to be explored (Sorted by f value)
        closed_set = set() #Nodes already evaluated
        start_node = Node(start) #The starting Node object (Current end effector position most likely)
        start_node.update_costs(0, self.heuristic(start, goal)) #Initialises the cost to be the absolute minimum (Straight line to end)
        heapq.heappush(open_set, start_node) #Add start node to the queue
        with ThreadPoolExecutor(max_workers=max_threads) as executor: #Initialise threadpool for simultaneous collision checks
            while open_set: #Loop for as long as their are nodes to be explored
                k+=1
                current_node = heapq.heappop(open_set) #Returns the highest priority node (That of the lowest cost)
                #if current_node.position == goal: #Check if the that node is at the goal position
                if np.linalg.norm(np.array(current_node.position) - np.array(goal)) < tol: #Check if final node is within tolerance around goal to counteract grid alignment errors
                    path = [] 
                    direction = []
                    while current_node: #Write the node tree to a list to be used
                        path.append(current_node.position)
                        direction.append(current_node.direction)
                        current_node = current_node.parent
                        if path[-1] != goal:
                            path.append(goal)
                    path = [tuple(p/20 for p in i) for i in path]
                    return path[::-1], direction[::-1]

                closed_set.add(current_node.position) #Add it to the list of explored nodes

                futures = {}
                neighbours = self.get_neighbours(current_node, step_size) #Get all neighbours of the explored node

                for neighbour_pos in neighbours: #For all neighbours, check collisions
                    if neighbour_pos in closed_set: #If neighbour already explored, ignore
                        continue
                    
                    #Submit collision checks to the thread pool
                    futures[executor.submit(self.collision_check, current_node.position, neighbour_pos)] = neighbour_pos

                for future in as_completed(futures): #Process each collision check as it ends
                    neighbour_pos = futures[future] #Store the neighbour position
                    if future.result():  #If collision is detected, ignore Node
                        continue
                    
                    dir = neighbours.index(neighbour_pos) #Use the index to represent the direction of the currently explored node from it's parent
                    g_cost = current_node.g + self.heuristic(current_node.position, neighbour_pos) #If node is valid however, calculate it's costs
                    neighbour_node = Node(neighbour_pos, current_node, direction=dir) #Create a new Node with the position of the current Node and keep the direction
                    neighbour_node.update_costs(g_cost, self.heuristic(neighbour_pos, goal)) #Update node costs

                    if neighbour_pos not in [n.position for n in open_set] or g_cost < neighbour_node.g: #If the neighbour is not in the open_set already, add it
                        heapq.heappush(open_set, neighbour_node)
                #logging.info(msg = f"Exploring node: {k}")
                tol = round((0.6 + int(k/2000)/20),1) #Increase tolerance by 0.05 if can't find path after 2000 searches
        return None

    def refined_theta_star(self, goal, start = None, max_threads=4, step_size=1, fn = False):
        path, dir = self.theta_star(goal, start, max_threads, step_size) #Get a normal path
        new_path = [] #Initialise important variables
        last_dir = None #Keeps track of the last variable value for comparison
        consecutive_dir = 0 #Keeps track of how many consecutive direction values there have been
        current_dir = 0 #Represent the currently compared direction value
        to_delete = [] #List to determine which nodes to delete, acts as a binary mask
        consecutives = [] #List to keep track of the consecutive chains of directions
        for i in range(len(dir)): #Loop for all directions in the list
            current_dir = dir[i] #Get the current direction
            if current_dir == last_dir: #If the current direction equals the last
                consecutive_dir +=1 #      then add one to the consecutive direction variable
            else: #                     Otherwise
                if consecutive_dir != 0: #If the variable isn't zero (accounting for an edge case)
                    consecutives.append(consecutive_dir) #Add it to the list of cariables
                consecutive_dir = 1 #Reset it back to one to start a new consecutive chain
            last_dir = current_dir #Set the last to the current for the next loop
        consecutives.append(consecutive_dir) #At the end, append the consecutive_dir to the list one last time to ensure it gets the last chain
        for teger in consecutives: #For each integer in consecutives
            if teger > 2: #If it is greater than 2 (i.e: 3 or more of the same direction in a row), it is considered a significant direction change
                to_delete.append(1) #Append 1 to the to_delete list
                for i in range(teger-2): #Then, append 0 for all nodes in between the first and the last node in the consecutive chain
                    to_delete.append(0) #i.e a chain of 5 will be 10001, keeping only the first and last element
                to_delete.append(1) #Cap off the zeros with another 1
            if teger < 3: #If less than 3, delete all the nodes
                for i in range(teger):
                    to_delete.append(0)
        if fn: #Keep first node? variable set by user, default False because the first node is usually the starting position of the end effector
            to_delete[0] = 1 #Set the first element to 1 to ensure we keep the starting position node
        else:
            to_delete[0] = 0 #Set the first element to 0 to ensure we remove the starting position node
        for i in range(len(to_delete)):
            if to_delete[i] == 1: #Iterate through the to_delete list, using it as a mask, if there is a 1, transfer the node to the new path list 
                new_path.append(path[i])
        new_path.append(path[-1])
        return new_path

    def plot_path(self,path, start, goal, col='blue'):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        if path:
            path = np.array(path)
            ax.plot(path[:, 0], path[:, 1], path[:, 2], color=col, linewidth=2, label='Path')
        ax.scatter(*start, color='green', s=100, label='Start')
        ax.scatter(*goal, color='red', s=100, label='Goal')
        ax.set_xlim(-1, 21)
        ax.set_ylim(-1, 21)
        ax.set_zlim(-1, 21)
        ax.grid()
        ax.legend()
        plt.show()

if __name__ == "__main__":
    r = rtb.models.UR3()
    start_point = (0, 0, 0)
    goal_point = (1, 1, 1)
    theta = ItzThetaStarPathing(r)
    path, _ = theta.theta_star(goal_point, start_point, max_threads=4)
    #print(path)
    #print(type(path[0]))
    ref_path = theta.refined_theta_star(goal_point, start_point, max_threads=4)
    #print(ref_path)
    theta.plot_path(path, start_point, goal_point)
    theta.plot_path(ref_path, start_point, goal_point, 'yellow')