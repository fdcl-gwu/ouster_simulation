import rospy
import os
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from datetime import datetime
from std_srvs.srv import Trigger, TriggerResponse

# Ensure the 'record' directory exists
save_dir = "gazebo_pc_record_full_12_42"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
    rospy.loginfo(f"Created directory: {save_dir}")

# Store latest pose and cloud
latest_pose = None
latest_cloud = None

def model_states_callback(model_msg):
    """ Store the latest model state for the target object. """
    global latest_pose
    try:
        index = model_msg.name.index('example')

        latest_pose = {
            'x': model_msg.pose[index].position.x,
            'y': model_msg.pose[index].position.y,
            'z': model_msg.pose[index].position.z,
            'qx': model_msg.pose[index].orientation.x,
            'qy': model_msg.pose[index].orientation.y,
            'qz': model_msg.pose[index].orientation.z,
            'qw': model_msg.pose[index].orientation.w
        }
    except ValueError:
        rospy.logwarn("example not found in model states.")

def cloud_callback(cloud_msg):
    """ Store the latest point cloud. """
    global latest_cloud
    latest_cloud = cloud_msg

def handle_cloud_request(req):
    """ Service handler: Return the latest point cloud when requested. """
    global latest_cloud, latest_pose

    if latest_cloud is None or latest_pose is None:
        rospy.logwarn("No valid pose or point cloud available.")
        return TriggerResponse(success=False, message="No point cloud captured.")

    # Convert PointCloud2 to Open3D format
    cloud_points = list(pc2.read_points(latest_cloud, field_names=("x", "y", "z"), skip_nans=True))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_points)

    # Generate timestamp for saving files
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename_ply = os.path.join(save_dir, f"pointcloud_{timestamp}.ply")
    filename_pose = os.path.join(save_dir, f"pointcloud_{timestamp}_pose.txt")

    # Save the point cloud as PLY
    o3d.io.write_point_cloud(filename_ply, pcd)

    # Save pose data to a text file
    with open(filename_pose, 'w') as f:
        f.write(f"Pose: {latest_pose}\n")

    rospy.loginfo(f"Point cloud saved to {filename_ply}, pose saved to {filename_pose}")

    return TriggerResponse(success=True, message=f"Point cloud saved: {filename_ply}")

if __name__ == "__main__":
    rospy.init_node('pose_cloud_service')

    # Subscribe to model states and point clouds
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback, queue_size=1)
    rospy.Subscriber("/os1_cloud_node/points", PointCloud2, cloud_callback, queue_size=1)

    # Provide a service for requesting the latest point cloud
    rospy.Service("/save_point_cloud", Trigger, handle_cloud_request)

    rospy.loginfo("Pose-Cloud Service is ready.")
    rospy.spin()
