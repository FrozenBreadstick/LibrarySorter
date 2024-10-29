import numpy as np
import random
import heapq
from concurrent.futures import ThreadPoolExecutor
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

class Node:
    def __init__(self, position):
        self.position = position
        self.parent = None
        self.cost = 0

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def steer(from_node, to_node, step_size):
    direction = np.array(to_node.position) - np.array(from_node.position)
    length = np.linalg.norm(direction)
    if length <= step_size:
        return to_node
    return Node(from_node.position + (direction / length) * step_size)

def collision_check(node1, node2, collision_func):
    return collision_func(node1.position, node2.position)

def rrt_star(start, goal, collision_func, max_threads=4, step_size=1, max_iterations=5000):
    start_node = Node(start)
    goal_node = Node(goal)
    nodes = [start_node]
    
    for _ in range(max_iterations):
        random_point = np.random.rand(3) * 10  # Generate a random point in 3D space
        nearest_node = min(nodes, key=lambda node: distance(node.position, random_point))
        new_node = steer(nearest_node, Node(random_point), step_size)
        print(_)
        if collision_check(nearest_node, new_node, collision_func):
            continue

        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + distance(nearest_node.position, new_node.position)
        nodes.append(new_node)

        neighbours = [node for node in nodes if distance(node.position, new_node.position) <= step_size * 2]

        with ThreadPoolExecutor(max_workers=max_threads) as executor:
            futures = {executor.submit(collision_check, neighbour, new_node, collision_func): neighbour for neighbour in neighbours}
            for future in futures:
                neighbour = futures[future]
                if future.result():  # If collision is detected
                    continue
                new_cost = new_node.cost + distance(new_node.position, neighbour.position)
                if new_cost < neighbour.cost:
                    neighbour.parent = new_node
                    neighbour.cost = new_cost

        if distance(new_node.position, goal_node.position) < step_size:
            goal_node.parent = new_node
            nodes.append(goal_node)
            break

    path = []
    current_node = goal_node if goal_node.parent else None
    while current_node:
        path.append(current_node.position)
        current_node = current_node.parent
    path.reverse()
    return path

# Collision check function that simulates a collision
def collision_func(p1, p2):
    # Simulate a 50% chance of collision
    return False#random.choice([True, False])  # True means collision detected, False means clear

def curve_profile(path):
    x = []
    y = []
    z = []
    for xp,yp,zp in path:
        x.append(xp)
        y.append(yp)
        z.append(zp)
    xdata = np.asarray(x)
    ydata = np.asarray(y)
    zdata = np.asarray(z)
    parameters, covariance = curve_fit(Gauss, xdata, zdata)
    fit_A = parameters[0]
    fit_B = parameters[1]
    fit_z = Gauss(xdata, fit_A, fit_B)

def Gauss(x, A, B):
    y = A*np.exp(-1*B*x**2)
    return y

# Example usage
start = (0, 0, 0)
goal = (10, 10, 10)
path = rrt_star(start, goal, collision_func, max_threads=4, step_size=1)

# Plotting the found path
def plot_path(path, start, goal):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if path:
        path = np.array(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color='blue', linewidth=2, label='Path')
    
    ax.scatter(*start, color='green', s=100, label='Start', marker='o')
    ax.scatter(*goal, color='red', s=100, label='Goal', marker='o')

    ax.set_title('RRT* Path Planning')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.legend()
    plt.show()

plot_path(path, start, goal)
curve_profile(path)