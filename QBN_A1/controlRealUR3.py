import roslibpy
import time

# Replace with the IP address of your Raspberry Pi running ROS
raspberry_pi_ip = '192.168.27.1'  # Example IP, change to your actual IP

# Create a client to connect to ROS
client = roslibpy.Ros(host=raspberry_pi_ip, port=9090)

try:
    # Start the connection
    client.run()

    # Subscribe to the joint states to monitor the robot's current joint configuration
    subscriber = roslibpy.Topic(client, '/ur/joint_states', 'sensor_msgs/JointState')
    subscriber.subscribe(lambda message: print(f"Joint State: {message}"))  # Print the joint states

    # Create a publisher to send joint trajectory commands
    control_topic = roslibpy.Topic(client, '/ur/scaled_pos_joint_traj_controller/command', 'trajectory_msgs/JointTrajectory')
    control_topic.advertise()

    # Create a JointTrajectory message to move the robot
    joint_trajectory_msg = {
        'joint_names': [
            'shoulder_pan_joint', 
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint', 
            'wrist_2_joint', 
            'wrist_3_joint'
        ],
        'points': [{
            'positions': [1.57, -1.57, 0.0, -1.57, 0.0, 0.0],
            'time_from_start': {'secs': 10, 'nsecs': 0}  # 5 seconds to reach the desired position
        }]
    }

    # Publish the trajectory message to move the robot
    print("Publishing joint trajectory command...")
    control_topic.publish(roslibpy.Message(joint_trajectory_msg))
    print("Joint trajectory command published.")
    time.sleep(1)
    


finally:
    # Ensure the client is properly terminated
    print("Terminating connection...")
    subscriber.unsubscribe()
    control_topic.unadvertise()
    client.terminate()
    print("Connection terminated.")
