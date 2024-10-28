# import numpy as np
# import heapq #Efficient list like equivalent for keeping track of item priority. Basically puts all items stored within a 'heap' in order of least to most priority, used for speed
# import matplotlib.pyplot as plt
# import trimesh
# from mpl_toolkits.mplot3d import Axes3D
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# from concurrent.futures import ThreadPoolExecutor
# from ir_support import *
# import time

# bannedspaces = [(1,0.5,0),(0.9,0.5,0),(0.8,0.5,0),(0.7,0.5,0),(0.6,0.5,0), (0.5,0.5,0), (0.4,0.5,0), (0.3,0.5,0), (0.2,0.5,0), (0.1,0.5,0), (0,0.5,0)]

# def collision(p1, p2, obj = None):
#     #NEED TO CHECK FOR COLLISION
#     for p in bannedspaces:
#         if p == p2:
#             return True
#     return False
#     #return np.random.choice([True, False], p=[0.1, 0.9])  #temp random collision return

# #Heuristic function for A* pathfinding
# def heuristic(p1, p2):
#     return np.linalg.norm(np.array(p1) - np.array(p2)) #"As the crow flies" distance between point 1 and point 2

# #A* with threaded collision checks (threading for speed)
# def astar_3d(start, goal, precision=1, threadnum=8): #precision = accuracy, i.e, the world coordinate frame is treated like a 3D grid, we specify how small that grid is here 
#     open_nodes = [] #Priority queue (cost, node) | Nodes that are worth exploring further during calculation
#     heapq.heappush(open_nodes, (0, start)) #Pushes the start node onto the heap with an initial cost score of 0
#     #Cost helps keep track of the most efficient path, longer paths will have a higher cost
#     cost_so_far = {start: 0} #Dict to store cost to get to each node in currently explored path
#     came_from = {start: None} #Dict to store representation of currently explored path
    
#     #List to be used for directional additions when exploring the "grid" space
#     inc = 1/(10**precision)
#     directions = [
#     # Primary axes movements
#     (inc, 0, 0), (-inc, 0, 0),
#     (0, inc, 0), (0, -inc, 0),
#     (0, 0, inc), (0, 0, -inc),
    
#     # 45-degree diagonals in 2D planes
#     (inc, inc, 0), (inc, -inc, 0),
#     (-inc, inc, 0), (-inc, -inc, 0),
#     (inc, 0, inc), (inc, 0, -inc),
#     (-inc, 0, inc), (-inc, 0, -inc),
#     (0, inc, inc), (0, inc, -inc),
#     (0, -inc, inc), (0, -inc, -inc),
    
#     # 45-degree diagonals in 3D
#     (inc, inc, inc), (inc, inc, -inc),
#     (inc, -inc, inc), (inc, -inc, -inc),
#     (-inc, inc, inc), (-inc, inc, -inc),
#     (-inc, -inc, inc), (-inc, -inc, -inc)
#     ]
    
#     with ThreadPoolExecutor(max_workers=threadnum) as executor: #Using the threadpoolexecutor (to create threads)
#         while open_nodes:
#             _, current = heapq.heappop(open_nodes)
            
#             #If goal reached, reconstruct path
#             if current == goal:
#                 path = []
#                 while current is not None:
#                     path.append(current)
#                     current = came_from[current]
#                 path.reverse()
#                 return path
            
#             #Check all possible movements
#             futures = {}
#             for dx, dy, dz in directions:
#                 neighbour = (round(current[0] + dx, precision), 
#                             round(current[1] + dy, precision), 
#                             round(current[2] + dz, precision))
                
#                 #Skip if already visited
#                 if neighbour in cost_so_far:
#                     continue
                
#                 #Submit collision checks to the thread pool
#                 future = executor.submit(collision, current, neighbour)
#                 futures[future] = neighbour
            
#             #Process collision check results
#             for future in futures:
#                 neighbour = futures[future]
#                 if future.result():  #Collision detected
#                     continue
                
#                 #Update costs and paths if no collision
#                 new_cost = cost_so_far[current] + heuristic(current, neighbour)
#                 if neighbour not in cost_so_far or new_cost < cost_so_far[neighbour]:
#                     cost_so_far[neighbour] = new_cost
#                     priority = new_cost + heuristic(neighbour, goal)
#                     heapq.heappush(open_nodes, (priority, neighbour))
#                     came_from[neighbour] = current
    
#     #No path found
#     return None

# def plot_3d_path(start, goal, path):
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.scatter(*start, color='green', s=100, label='Start')
#     ax.scatter(*goal, color='red', s=100, label='Goal')
#     b = np.array(bannedspaces)
#     ax.plot(b[:, 0], b[:, 1], b[:, 2], color= 'red', linewidth = 1, label = 'Wall')
#     if path:
#         path = np.array(path)
#         ax.plot(path[:, 0], path[:, 1], path[:, 2], color='blue', linewidth=2, label='Path')
#     else:
#         print("No path found.")
#     plt.show()

# start_point = (0.0, 0.0, 0.0)
# goal_point = (1.0, 1.0, 0)

# path = astar_3d(start_point, goal_point)
# plot_3d_path(start_point, goal_point, path)

# # myobj = trimesh.load_mesh("temp/cube.ply", enable_post_processing=True, solid=True) # Import Objects
# # fig = plt.figure()
# # ax = fig.add_subplot(111, projection='3d')
# # verts = myobj.vertices
# # faces = myobj.faces
# # i = 0
# # for face in faces:
# #     # Get the vertices for this face
# #     face_vertices = verts[face]
# #     # Create a polygon for the face
# #     if i < 6:
# #         poly = Poly3DCollection([face_vertices], alpha=0.5, edgecolor='k')
# #         ax.add_collection3d(poly)
# #     i+=1
# # # ax.scatter(faces[:,0], faces[:,1], faces[:,2], color = 'red', linewidth = 1)
# # plt.show()
# # for i in range(10):
# #     print("\n")

#-----------------------------------------------------------------------------#


#RRT* Implementation
# import numpy as np
# import random
# import heapq
# from concurrent.futures import ThreadPoolExecutor
# import matplotlib.pyplot as plt

# class Node:
#     def __init__(self, position):
#         self.position = position
#         self.parent = None
#         self.cost = 0

# def distance(p1, p2):
#     return np.linalg.norm(np.array(p1) - np.array(p2))

# def steer(from_node, to_node, step_size):
#     direction = np.array(to_node.position) - np.array(from_node.position)
#     length = np.linalg.norm(direction)
#     if length <= step_size:
#         return to_node
#     return Node(from_node.position + (direction / length) * step_size)

# def collision_check(node1, node2, collision_func):
#     return collision_func(node1.position, node2.position)

# def rrt_star(start, goal, collision_func, max_threads=4, step_size=0.5, max_iterations=5000):
#     start_node = Node(start)
#     goal_node = Node(goal)
#     nodes = [start_node]
    
#     for _ in range(max_iterations):
#         random_point = np.random.rand(3) * 10  # Generate a random point in 3D space
#         nearest_node = min(nodes, key=lambda node: distance(node.position, random_point))
#         new_node = steer(nearest_node, Node(random_point), step_size)
#         print(_)
#         if collision_check(nearest_node, new_node, collision_func):
#             continue

#         new_node.parent = nearest_node
#         new_node.cost = nearest_node.cost + distance(nearest_node.position, new_node.position)
#         nodes.append(new_node)

#         neighbours = [node for node in nodes if distance(node.position, new_node.position) <= step_size * 2]

#         with ThreadPoolExecutor(max_workers=max_threads) as executor:
#             futures = {executor.submit(collision_check, neighbour, new_node, collision_func): neighbour for neighbour in neighbours}
#             for future in futures:
#                 neighbour = futures[future]
#                 if future.result():  # If collision is detected
#                     continue
#                 new_cost = new_node.cost + distance(new_node.position, neighbour.position)
#                 if new_cost < neighbour.cost:
#                     neighbour.parent = new_node
#                     neighbour.cost = new_cost

#         if distance(new_node.position, goal_node.position) < step_size:
#             goal_node.parent = new_node
#             nodes.append(goal_node)
#             break

#     path = []
#     current_node = goal_node if goal_node.parent else None
#     while current_node:
#         path.append(current_node.position)
#         current_node = current_node.parent
#     path.reverse()
#     return path

# # Collision check function that simulates a collision
# def collision_func(p1, p2):
#     # Simulate a 50% chance of collision
#     return False#random.choice([True, False])  # True means collision detected, False means clear

# # Example usage
# start = (0, 0, 0)
# goal = (1, 1, 1)
# path = rrt_star(start, goal, collision_func, max_threads=4, step_size=0.1)

# # Plotting the found path
# def plot_path(path, start, goal):
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     if path:
#         path = np.array(path)
#         ax.plot(path[:, 0], path[:, 1], path[:, 2], color='blue', linewidth=2, label='Path')
    
#     ax.scatter(*start, color='green', s=100, label='Start', marker='o')
#     ax.scatter(*goal, color='red', s=100, label='Goal', marker='o')

#     ax.set_title('RRT* Path Planning')
#     ax.set_xlabel('X-axis')
#     ax.set_ylabel('Y-axis')
#     ax.set_zlabel('Z-axis')
#     ax.legend()
#     plt.show()

# plot_path(path, start, goal)

#------------------------------------------------------------#

#Theta* Implementation
import numpy as np
import heapq #Heapq is a list-like alternative that organizes items based on a score, allows for quick and efficient access to smallest and highest priority items. Keeps items stored in order
import random
from concurrent.futures import ThreadPoolExecutor, as_completed
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def collision_check(p1, p2):
    return False #random.choice([True, False])  # 50% chance of collision

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

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b)) #Simple calculation, distance between two nodes

def get_neighbours(node, step_size=0.5): #Get's all possible neighbour positions around a node
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

def theta_star(start, goal, max_threads=4, step_size=1): #Method for the Theta# implementation
    open_set = [] #Nodes to be explored (Sorted by f value)
    closed_set = set() #Nodes already evaluated
    start_node = Node(start) #The starting Node object (Current end effector position most likely)
    start_node.update_costs(0, heuristic(start, goal)) #Initialises the cost to be the absolute minimum (Straight line to end)
    heapq.heappush(open_set, start_node) #Add start node to the queue

    with ThreadPoolExecutor(max_workers=max_threads) as executor: #Initialise threadpool for simultaneous collision checks
        while open_set: #Loop for as long as their are nodes to be explored
            current_node = heapq.heappop(open_set) #Returns the highest priority node (That of the lowest cost)
            if current_node.position == goal: #Check if the that node is at the goal position
                path = [] 
                direction = []
                while current_node: #Write the node tree to a list to be used
                    path.append(current_node.position)
                    direction.append(current_node.direction)
                    current_node = current_node.parent
                return path[::-1], direction[::-1]

            closed_set.add(current_node.position) #Add it to the list of explored nodes

            futures = {}
            neighbours = get_neighbours(current_node, step_size) #Get all neighbours of the explored node

            for neighbour_pos in neighbours: #For all neighbours, check collisions
                if neighbour_pos in closed_set: #If neighbour already explored, ignore
                    continue
                
                #Submit collision checks to the thread pool
                futures[executor.submit(collision_check, current_node.position, neighbour_pos)] = neighbour_pos

            for future in as_completed(futures): #Process each collision check as it ends
                neighbour_pos = futures[future] #Store the neighbour position
                if future.result():  #If collision is detected, ignore Node
                    continue
                
                dir = neighbours.index(neighbour_pos) #Use the index to represent the direction of the currently explored node from it's parent
                g_cost = current_node.g + heuristic(current_node.position, neighbour_pos) #If node is valid however, calculate it's costs
                neighbour_node = Node(neighbour_pos, current_node, direction=dir) #Create a new Node with the position of the current Node and keep the direction
                neighbour_node.update_costs(g_cost, heuristic(neighbour_pos, goal)) #Update node costs

                if neighbour_pos not in [n.position for n in open_set] or g_cost < neighbour_node.g: #If the neighbour is not in the open_set already, add it
                    heapq.heappush(open_set, neighbour_node)

    return None

def refined_theta_star(start, goal, max_threads=4, step_size=1):
    path, dir = theta_star(start, goal, max_threads, step_size)
    new_path = []
    last_dir = None
    consecutive_dir = 0
    current_dir = 0
    to_delete = []
    consecutives = []
    for i in range(len(dir)):
        current_dir = dir[i]
        if current_dir == last_dir:
            consecutive_dir +=1
        else:
            if consecutive_dir != 0:
                consecutives.append(consecutive_dir)
            consecutive_dir = 1
        last_dir = current_dir
    consecutives.append(consecutive_dir)
    for teger in consecutives:
        if teger > 2:
            to_delete.append(1)
            for i in range(teger-2):
                to_delete.append(0)
            to_delete.append(1)
        if teger < 3:
            for i in range(teger):
                to_delete.append(0)
    to_delete[0] = 1
    for i in range(len(to_delete)):
        if to_delete[i] == 1:
            new_path.append(path[i])
    new_path.append(path[-1])
    return new_path

def plot_path(path, start, goal, col='blue'):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    if path:
        path = np.array(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color=col, linewidth=2, label='Path')
    ax.scatter(*start, color='green', s=100, label='Start')
    ax.scatter(*goal, color='red', s=100, label='Goal')
    ax.set_xlim(-1, 11)
    ax.set_ylim(-1, 11)
    ax.set_zlim(-1, 11)
    ax.grid()
    ax.legend()
    plt.show()

start_point = (0, 0, 0)
goal_point = (10, 10, 5)

path, _ = theta_star(start_point, goal_point, max_threads=4)
ref_path = refined_theta_star(start_point, goal_point, max_threads=4)
plot_path(path, start_point, goal_point)
plot_path(ref_path, start_point, goal_point, 'green')


#---------------------------#
#Refining Algorithm Testing
# dir = [0,0,1,1,1,1,1,3,3,1,1,4,5,0,0,2,2,2,2,0,0]
# path = ['a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u']

# new_path = []
# last_dir = None
# consecutive_dir = 1
# current_dir = 0
# to_delete = []
# long_consecutives = []
# for i in range(len(dir)):
#     if last_dir is not None:
#         current_dir = dir[i]
#         if current_dir == last_dir:
#             consecutive_dir += 1
#         else:
#             if consecutive_dir == 1:
#                 to_delete.append(i-1)
#             elif consecutive_dir < 3:
#                 for p in ((consecutive_dir-1),0):
#                     to_delete.append(i-p-1)
#             else:
#                 for p in ((consecutive_dir-2),1):
#                     long_consecutives.append(i-p-1)
#             consecutive_dir = 1
#         if(i == (len(dir)-1)):
#             if consecutive_dir == 1:
#                 to_delete.append(i)
#             elif consecutive_dir < 3:
#                 for p in ((consecutive_dir-1),0):
#                     to_delete.append(i-p)
#             else:
#                 for p in ((consecutive_dir-2),1):
#                     long_consecutives.append(i-p)
#     last_dir = current_dir
# new_path.append(path[0])
# for i in range(len(path)):
#     if i not in to_delete:
#         new_path.append(path[i])
# new_path.append(path[-1])
# print(long_consecutives)
# print(new_path)

