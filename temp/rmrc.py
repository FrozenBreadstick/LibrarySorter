import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
from swift import Swift
import math
import spatialgeometry as sg
import sys
sys.path.append('../LibrarySorter')
from Models.Robots.Itzamna import Itzamna
import time
from math import pi

def joint_limit_avoidance(q, q_min, q_max, gain=0.1):
    avoidance = np.zeros(len(q))
    for i, qi in enumerate(q):
        if qi < q_min[i] + 0.1 * (q_max[i] - q_min[i]):  # Close to lower limit
            avoidance[i] = gain * (q_min[i] - qi)
        elif qi > q_max[i] - 0.1 * (q_max[i] - q_min[i]):  # Close to upper limit
            avoidance[i] = gain * (q_max[i] - qi)
    return avoidance

q_min = np.array([0.3, -1.2, -pi/2, -pi/2, -pi/2, -pi/2, -pi/2])  # Example joint limits
q_max = np.array([2.7, -0.2, pi/2, pi/2, pi/2, pi/2, pi/2])

r = Itzamna()
r.q = [0.4, 0, 0, 0, 0, 0, 0]
rt = rtb.models.Panda()
rt.q = rt.qr

# Make a new environment and add our robot
env = Swift()
env.launch(realtime=True)
r.add_to_env(env)
# env.add(r)

# Step the sim to view the robot in this configuration
env.step()
print("here")

# Specify our desired end-effector velocity
ev = [0, -0.01, 0, 0.0, 0, 0.0]

# Specify our timestep
dt = 0.05

# Run the simulation for 5 seconds
for _ in range(100):
    J = r.jacob0(r.q)
    lambda_damping = 0.01
    J_pinv = np.linalg.inv(J.T @ J + (lambda_damping ** 2) * np.eye(J.shape[1])) @ J.T
    p = J_pinv @ ev
    c = r.q
    q = []
    avoidance = joint_limit_avoidance(r.q, q_min, q_max)
    for i in range(len(c)):
        q.append(c[i]+p[i]+avoidance[i])
    
    r.q = q

    env.step()

    # Step the simulator by dt seconds
    time.sleep(dt)