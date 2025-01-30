import rospy
import os
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from datetime import datetime
import message_filters

# Ensure the 'record' directory exists
save_dir = "gazebo_pc_record"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
    rospy.loginfo(f"Created directory: {save_dir}")

most_recent_pose = None

def synchronized_callback(model_msg, cloud_msg):
    global most_recent_pose
    try:
        # Find the index of 'example'
        index = model_msg.name.index('example')

        # Extract position and orientation
        position = model_msg.pose[index].position
        orientation = model_msg.pose[index].orientation

        pose_data = {
            'x': position.x,
            'y': position.y,
            'z': position.z,
            'qx': orientation.x,
            'qy': orientation.y,
            'qz': orientation.z,
            'qw': orientation.w
        }

        if most_recent_pose == pose_data:
            # prints 9 times per second (since cloud is at 10hz)
            rospy.loginfo("Pose already synchronized.")
            return
        
        # rospy.loginfo(f"Pose Synchronized: {pose_data}")

        # Convert ROS PointCloud2 message to list of points
        cloud_points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))

        # Convert to Open3D PointCloud
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
            f.write(f"Pose: {pose_data}\n")

        rospy.loginfo(f"Point cloud saved to {filename_ply}, pose saved to {filename_pose}")
        most_recent_pose = pose_data
        
    except ValueError:
        rospy.logwarn("example not found in model states.")

if __name__ == "__main__":
    rospy.init_node('pose_cloud_recorder', anonymous=True)

    # Create subscribers with message_filters for synchronization
    model_states_sub = message_filters.Subscriber("/gazebo/model_states", ModelStates)
    point_cloud_sub = message_filters.Subscriber("/os1_cloud_node/points", PointCloud2)

    # Approximate Time Synchronization with allow_headerless=True
    ts = message_filters.ApproximateTimeSynchronizer([model_states_sub, point_cloud_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ts.registerCallback(synchronized_callback)

    rospy.loginfo("Synchronizing and recording pose and point clouds in 'gazebo_pc_record/' directory...")

    rospy.spin()
