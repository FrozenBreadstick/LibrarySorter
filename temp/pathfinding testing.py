import numpy as np
import heapq #Efficient list like equivalent for keeping track of item priority. Basically puts all items stored within a 'heap' in order of least to most priority, used for speed
import matplotlib.pyplot as plt
import spatialgeometry as geometry
from mpl_toolkits.mplot3d import Axes3D
from concurrent.futures import ThreadPoolExecutor
from ir_support import *

bannedspaces = [(1,0.5,0),(0.9,0.5,0),(0.8,0.5,0),(0.7,0.5,0),(0.6,0.5,0), (0.5,0.5,0), (0.4,0.5,0), (0.3,0.5,0), (0.2,0.5,0), (0.1,0.5,0), (0,0.5,0)]

def collision(p1, p2, obj = None):
    #NEED TO CHECK FOR COLLISION
    for p in bannedspaces:
        if p == p2:
            return True
    return False
    #return np.random.choice([True, False], p=[0.1, 0.9])  #temp random collision return

#Heuristic function for A* pathfinding
def heuristic(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2)) #"As the crow flies" distance between point 1 and point 2

#A* with threaded collision checks (threading for speed)
def astar_3d(start, goal, precision=1, threadnum=8): #precision = accuracy, i.e, the world coordinate frame is treated like a 3D grid, we specify how small that grid is here 
    open_nodes = [] #Priority queue (cost, node) | Nodes that are worth exploring further during calculation
    heapq.heappush(open_nodes, (0, start)) #Pushes the start node onto the heap with an initial cost score of 0
    #Cost helps keep track of the most efficient path, longer paths will have a higher cost
    cost_so_far = {start: 0} #Dict to store cost to get to each node in currently explored path
    came_from = {start: None} #Dict to store representation of currently explored path
    
    #List to be used for directional additions when exploring the "grid" space
    inc = 1/(10**precision)
    directions = [
        (inc, 0, 0), (-inc, 0, 0),
        (0, inc, 0), (0, -inc, 0),
        (0, 0, inc), (0, 0, -inc)
    ]
    
    with ThreadPoolExecutor(max_workers=threadnum) as executor: #Using the threadpoolexecutor (to create threads)
        while open_nodes:
            _, current = heapq.heappop(open_nodes)
            
            #If goal reached, reconstruct path
            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
            
            #Check all possible movements
            futures = {}
            for dx, dy, dz in directions:
                neighbor = (round(current[0] + dx, precision), 
                            round(current[1] + dy, precision), 
                            round(current[2] + dz, precision))
                
                #Skip if already visited
                if neighbor in cost_so_far:
                    continue
                
                #Submit collision checks to the thread pool
                future = executor.submit(collision, current, neighbor)
                futures[future] = neighbor
            
            #Process collision check results
            for future in futures:
                neighbor = futures[future]
                if future.result():  #Collision detected
                    continue
                
                #Update costs and paths if no collision
                new_cost = cost_so_far[current] + heuristic(current, neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(open_nodes, (priority, neighbor))
                    came_from[neighbor] = current
    
    #No path found
    return None

def plot_3d_path(start, goal, path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(*start, color='green', s=100, label='Start')
    ax.scatter(*goal, color='red', s=100, label='Goal')
    b = np.array(bannedspaces)
    ax.plot(b[:, 0], b[:, 1], b[:, 2], color= 'red', linewidth = 1, label = 'Wall')
    if path:
        path = np.array(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color='blue', linewidth=2, label='Path')
    else:
        print("No path found.")
    plt.show()

start_point = (0.0, 0.0, 0.0)
goal_point = (1.0, 1.0, 0)

path = astar_3d(start_point, goal_point)
plot_3d_path(start_point, goal_point, path)
