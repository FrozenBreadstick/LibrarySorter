
# Python Implementation

import roslibpy
import time

class URControl:
    def __init__(self, raspberry_pi_ip, port=9090):
        self.client = roslibpy.Ros(host=raspberry_pi_ip, port=port)
        self.open_srv = roslibpy.Service(self.client, '/onrobot/open', 'std_srvs/Trigger')
        self.close_srv = roslibpy.Service(self.client, '/onrobot/close', 'std_srvs/Trigger')
        
        self.joint_states = None  # Variable to store joint states
        self.subscriber = None
        self.control_topic = None

    def start_client(self):
        self.client.run()

    def publish_trajectory_command(self, positions, time_from_start):
        self.control_topic = roslibpy.Topic(self.client, '/ur/scaled_pos_joint_traj_controller/command', 'trajectory_msgs/JointTrajectory')
        self.control_topic.advertise()

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
                'positions': positions,
                'time_from_start': {'secs': time_from_start, 'nsecs': 0}
            }]
        }

        # Publish the trajectory message to move the robot
        print("Publishing joint trajectory command...")
        self.control_topic.publish(roslibpy.Message(joint_trajectory_msg))
        print("Joint trajectory command published.")

    def open_gripper(self):
        rq = roslibpy.ServiceRequest()
        self.open_srv.call(rq)

    def close_gripper(self):
        rq = roslibpy.ServiceRequest()
        self.close_srv.call(rq)

    def stop_client(self):
        self.client.terminate()

# Example of usage
if __name__ == '__main__':
    raspberry_pi_ip = '192.168.27.1'  # Replace with your Raspberry Pi's IP

    ur_control = URControl(raspberry_pi_ip)
    ur_control.start_client()
    ur_control.close_gripper()
    
    # # Input: Joint positions and time from start
    # joint_positions = [1.57, -1.57, 0.0, -1.57, 0.0, 0.0]  # Example positions, modify as needed
    # time_from_start = 5  # Example time in seconds, modify as needed
    
    # # Publish joint trajectory command with input
    # ur_control.publish_trajectory_command(joint_positions, time_from_start)
    # time.sleep(8)  # Allow some time for publishing
    

    # joint_positions = finalq=[-1.57, -1.57, 0.0, -1.57, 1.57, 0]  
    # time_from_start = 5
    # ur_control.publish_trajectory_command(joint_positions, time_from_start)
    # time.sleep(8)  # Allow some time for publishing 

    # joint_positions = finalq=[-1.57, -1.57, 0.0, -1.57, -1.57, 0]  
    # time_from_start = 3
    # ur_control.publish_trajectory_command(joint_positions, time_from_start)
    # time.sleep(5)  # Allow some time for publishing  

    
    # Stop the client connection
    ur_control.open_gripper()
    ur_control.stop_client()

