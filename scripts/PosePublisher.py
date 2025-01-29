import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
import numpy as np
import os
from scipy.spatial.transform import Rotation
from scipy.linalg import expm
import PoseScatterGenerator as PoseScatterGenerator

def set_model_state(C, R_list):
    # Begin the publishing to the simulation:
    print("Publishing model states to Gazebo...")

    # Create a publisher to the /gazebo/set_model_state topic
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.init_node('set_model_state_node', anonymous=True)

    rospy.sleep(0.5)  # Allow some time for publisher to register

    # Define the model state
    model_state = ModelState()
    model_state.model_name = "example"  # The name of the model from /gazebo/model_states
    model_state.reference_frame = "world"

    for i in range(10):
        # Set the desired position
        model_state.pose.position.x = C[i][0]
        model_state.pose.position.y = C[i][1]
        model_state.pose.position.z = C[i][2]

        # convert R to quaternion
        q = PoseScatterGenerator.to_quaternion(R_list[i])

        # Set the desired orientation (quaternion)
        model_state.pose.orientation.x = q[1]
        model_state.pose.orientation.y = q[2]
        model_state.pose.orientation.z = q[3]
        model_state.pose.orientation.w = q[0]

        rospy.loginfo("Publishing model state to Gazebo...")

        # Publish the message repeatedly to ensure it is received
        pub.publish(model_state)
        rospy.sleep(1) # sleep for 0.1 seconds
        rospy.loginfo("Model state set successfully.")

    rospy.loginfo("Model state set successfully.")

if __name__ == "__main__":
    try:
        C, F, CF, R_list = PoseScatterGenerator.generate_scatter_data()
        
        # Create a publisher to the /gazebo/set_model_state topic, sending over computed poses
        set_model_state(C, R_list)
    except rospy.ROSInterruptException:
        pass