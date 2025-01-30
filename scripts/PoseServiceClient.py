import rospy
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Trigger
import PoseScatterGenerator as PoseScatterGenerator

def set_model_state(C, R_list):
    """ Publishes poses to Gazebo and requests a point cloud after each update. """
    rospy.init_node('set_model_state_node', anonymous=True)
    
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    rospy.sleep(0.5)  # Allow time for the publisher to initialize

    # Wait for the point cloud service to become available
    rospy.wait_for_service('/save_point_cloud')
    save_point_cloud = rospy.ServiceProxy('/save_point_cloud', Trigger)

    model_state = ModelState()
    model_state.model_name = "example"
    model_state.reference_frame = "world"

    for i in range(len(C)):
        # Set position
        model_state.pose.position.x = C[i][0]
        model_state.pose.position.y = C[i][1]
        model_state.pose.position.z = C[i][2]

        # Convert R to quaternion
        q = PoseScatterGenerator.to_quaternion(R_list[i])

        # Set orientation
        model_state.pose.orientation.x = q[1]
        model_state.pose.orientation.y = q[2]
        model_state.pose.orientation.z = q[3]
        model_state.pose.orientation.w = q[0]

        rospy.loginfo(f"Publishing pose {i} to Gazebo...")
        pub.publish(model_state)

        # Allow time for Gazebo to update
        rospy.sleep(1)

        # Request the point cloud for the current pose. Note: the server 
        # will do the actual saving of the point cloud and pose.
        try:
            response = save_point_cloud()
            if response.success:
                rospy.loginfo(f"Point cloud {i} captured successfully.")
            else:
                rospy.logwarn(f"Failed to capture point cloud for pose {i}.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    rospy.loginfo("All poses processed.")

if __name__ == "__main__":
    try:
        # Generate scatter data
        C, F, CF, F_updated, R_list = PoseScatterGenerator.generate_scatter_data()
        
        # Publish poses and request corresponding point clouds
        set_model_state(C, R_list)
    except rospy.ROSInterruptException:
        pass
