from bagpy import bagreader
import pandas as pd
import os


def read_bag(path): 
    '''
    Function to convert ros .bag files to a text file for processing and analysis.
    :param path: relative path to .bag file.
    '''

    bag = bagreader(path)
    print(bag.topic_table)
    joint_state = bag.message_by_topic('/joint_states')
    joint_data = pd.read_csv(joint_state)
    df = pd.DataFrame(joint_data)
    cols = [6,7,8,9,10,11]  # select columns
    df = df[df.columns[cols]]
    
    output_dir= '/home/qbn_legion_ubun20/Desktop/IR_QBN/IR_py3.10.11/Assignment_1'
    output_file = os.path.join(output_dir, 'bag_q.txt')
    # Save the DataFrame to a text file
    df.to_csv('bag_q.txt', index=False)  

def move_from_text(path):
    '''
    Function to move the robot to the joint states specified in the text file.
    :param path: relative path to the text file.
    '''
    r = UR3()
    env = swift.Swift()
    env.launch(realtime=True)
    r.add_to_env(env)

    # Read the text file
    moveData = pd.read_csv(path)
    q_matrix=moveData.values
    # Remove the first row of the data
    print(np.shape(q_matrix))
    q_matrix = moveData.values[1:, :]  # Slicing to skip the first row
    print(np.shape(q_matrix))
    
    for i in range(0, len(q_matrix), 10):  # Step of 10 in the range
        
        r.q = q_matrix[i]  # Set the robot joint configuration
        env.step(0.02)