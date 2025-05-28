import rospy
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

def move_sensor_and_capture(base_position, base_quaternion, rpy_offset_deg=(0, 0, 0), translation_offset=(0, 0, 0), model_name="example"):
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.wait_for_service('/save_point_cloud')
    save_service = rospy.ServiceProxy('/save_point_cloud', Trigger)

    # Base rotation
    r_base = R.from_quat(base_quaternion)
    delta_rpy = np.radians(rpy_offset_deg)
    r_offset = R.from_euler('xyz', delta_rpy)
    r_final = r_offset * r_base
    q_final = r_final.as_quat()

    # Final pose
    tx, ty, tz = translation_offset
    pose = Pose()
    pose.position.x = base_position[0] + tx
    pose.position.y = base_position[1] + ty
    pose.position.z = base_position[2] + tz
    pose.orientation.x = q_final[0]
    pose.orientation.y = q_final[1]
    pose.orientation.z = q_final[2]
    pose.orientation.w = q_final[3]

    rospy.sleep(1.0)
    model_state = ModelState()
    model_state.model_name = model_name
    model_state.reference_frame = "world"
    model_state.pose = pose

    pub.publish(model_state)
    rospy.loginfo(f"Published pose with RPY offset: {rpy_offset_deg}, translation offset: {translation_offset}")
    rospy.sleep(1.0)

    try:
        response = save_service()
        if response.success:
            rospy.loginfo("Point cloud successfully saved.")
        else:
            rospy.logwarn("Failed to save point cloud: " + response.message)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    import numpy as np
    import open3d as o3d
    from glob import glob
    import os

    rospy.init_node('move_sensor_and_capture', anonymous=True)

    base_position = (-1.6712, -4.7926, 2.7962)
    base_quaternion = [0.02264919345024868, 0.07625865133688305, 0.5654648297221784, 0.8209270116640094]  # [x, y, z, w]
    rpy_offset_deg = (0.0, 0, 0)
    translation_offset = (0, 0, 0.0)

    move_sensor_and_capture(base_position, base_quaternion, rpy_offset_deg, translation_offset)
    
    
    # === Post-process most recent scan ===
    gazebo_cloud_dir = "/home/fdcl/Ouster/gazebo_ws_fdcl/src/ouster_simulation/gazebo_pc_record_os0_rev06-32_box_noise"
    corrected_cloud_dir = "/home/fdcl/Ouster/gazebo_ws_fdcl/src/ouster_simulation/pose_setter_output"

    saved_files = sorted(glob(os.path.join(gazebo_cloud_dir, "*.ply")), key=os.path.getmtime)

    if not saved_files:
        print("No .ply files found in Gazebo directory to correct.")
    else:
        latest_file = saved_files[-1]
        corrected_file = os.path.join(corrected_cloud_dir, "000001.ply")  # Or dynamically rename if needed

        try:
            cloud = o3d.io.read_point_cloud(latest_file)

            # Convert Open3D points to NumPy
            points = np.asarray(cloud.points)
            # Save corrected cloud
            o3d.io.write_point_cloud(corrected_file, cloud)
            print(f"Corrected scan saved as: {corrected_file}")

        except Exception as e:
            print(f"Failed to correct and save scan: {e}")
