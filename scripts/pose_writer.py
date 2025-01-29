# If pose_writer hangs, make sure you're not in a conda environment. Then source setup.bash again.

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

def set_model_state():
    # Create a publisher to the /gazebo/set_model_state topic
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    rospy.init_node('set_model_state_node', anonymous=True)

    rospy.sleep(1)  # Allow some time for publisher to register

    # Define the model state
    model_state = ModelState()
    model_state.model_name = "example"  # The name of the model from /gazebo/model_states
    model_state.reference_frame = "world"

    # Set the desired position
    model_state.pose.position.x = 1.86964867983716
    model_state.pose.position.y = -3.028062835607197
    model_state.pose.position.z = 2.7451070230083827

    # Set the desired orientation (quaternion)
    model_state.pose.orientation.x = -0.1158743065148767
    model_state.pose.orientation.y = 0.2547179622488181
    model_state.pose.orientation.z = 0.3975362205400908
    model_state.pose.orientation.w = 0.8738746238198054

    rospy.loginfo("Publishing model state to Gazebo...")

    # Publish the message repeatedly to ensure it is received
    rate = rospy.Rate(10)  # 10 Hz
    # for _ in range(10):
    pub.publish(model_state)
    rate.sleep()

    rospy.loginfo("Model state set successfully.")

if __name__ == "__main__":
    try:
        set_model_state()
    except rospy.ROSInterruptException:
        pass