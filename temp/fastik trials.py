from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Create the robot chain using the given DH parameters
robot_chain = Chain(name='custom_robot', links=[
    OriginLink(),  # Base link

    # Prismatic joints
    DHLink(
        name="prismatic_1",
        d=0,
        a=0,
        alpha=np.pi / 2,
        theta=np.pi / 2,  # DH parameter theta
        joint_type="prismatic",
        bounds=(-1, 1)  # qlim for prismatic joint
    ),
    DHLink(
        name="prismatic_2",
        d=0,
        a=0.167391,  # DH parameter a
        alpha=0,
        theta=0,  # DH parameter theta
        joint_type="prismatic",
        bounds=(-1, 1)  # qlim for prismatic joint
    ),

    # Revolute joints
    DHLink(
        name="revolute_1",
        d=0,  # DH parameter d
        a=0.5284043,  # DH parameter a
        alpha=0,  # DH parameter alpha
        theta=0,  # Offset theta
        joint_type="revolute",
        bounds=(-2 * np.pi, 2 * np.pi)  # qlim for revolute joint
    ),
    DHLink(
        name="revolute_2",
        d=0,  # DH parameter d
        a=0.397998,  # DH parameter a
        alpha=np.pi / 2,  # DH parameter alpha
        theta=0,  # Offset theta
        joint_type="revolute",
        bounds=(-2 * np.pi, 2 * np.pi)  # qlim for revolute joint
    ),
    DHLink(
        name="revolute_3",
        d=0,  # DH parameter d
        a=0.397998,  # DH parameter a
        alpha=0,  # DH parameter alpha
        theta=0,  # Offset theta
        joint_type="revolute",
        bounds=(-2 * np.pi, 2 * np.pi)  # qlim for revolute joint
    ),
    DHLink(
        name="revolute_4",
        d=0,  # DH parameter d
        a=0,  # DH parameter a
        alpha=np.pi / 2,  # DH parameter alpha
        theta=np.pi / 2,  # Offset theta
        joint_type="revolute",
        bounds=(-2 * np.pi, 2 * np.pi)  # qlim for revolute joint
    ),
    DHLink(
        name="revolute_5",
        d=0.307999,  # DH parameter d
        a=0,  # DH parameter a
        alpha=np.pi / 2,  # DH parameter alpha
        theta=-np.pi / 2,  # Offset theta
        joint_type="revolute",
        bounds=(-2 * np.pi, 2 * np.pi)  # qlim for revolute joint
    )
])



# Define a function to plot the robot
def plot_robot(robot_chain, joint_angles):
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Perform forward kinematics to get the positions of each joint
    frames = robot_chain.forward_kinematics(joint_angles, full_kinematics=True)
    
    # Extract the coordinates of each joint
    x_coords = [frame[0, 3] for frame in frames]
    y_coords = [frame[1, 3] for frame in frames]
    z_coords = [frame[2, 3] for frame in frames]
    
    # Plot the links between the joints
    ax.plot(x_coords, y_coords, z_coords, '-o', label='Robot Links')
    
    # Set plot limits for better visualization
    ax.set_xlim(-1, 1.5)
    ax.set_ylim(-1, 1.5)
    ax.set_zlim(-1, 1.5)
    
    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Robot Visualization')
    
    # Display the plot
    plt.legend()
    plt.show()

# Example joint angles (modify as needed)
joint_angles = [0, 0, 0, 0, 0, 0, 0]

# Plot the robot
plot_robot(robot_chain, joint_angles)
