# Simulating Ouster LiDAR in Gazebo
## Ouster Example

This repository is an adaptation of the [ouster_example](https://github.com/wilselby/ouster_example) repository. For IMU simulation, requires cloning the hector_gazebo_plugins package (see below). 
- NOTE: For code simplicity, several unnecessary directories from the original fork were removed. Refer to old commits for those details.

### Prerequisites

Make sure you have the following software installed:

- ROS Noetic
- Gazebo 11.14.0

### Installation

1. Create a catkin workspace:

    ```bash
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    ```

2. Clone the required repositories:

    ```bash
    git clone https://github.com/fdcl-gwu/ouster_simulation.git
    git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git hector_gazebo_plugins
    ```

3. Build the workspace:

    ```bash
    cd ..
    catkin build
    ```

### Usage

1. Source the workspace:

    ```bash
    source devel/setup.bash
    ```

2. Launch the Ouster simulation:

    ```bash
    roslaunch ouster_description os1_world.launch -v
    ```

### Modifications for OS0-32 rev0 specifications:
- `origin`: *
- `parent`: base_link
- `name`: os1_sensor
- `topic_points`: /os1_cloud_node/points
- `topic_imu`: /os1_cloud_node/imu
- `hz`: 10
- `lasers`: 32
- `samples`: 512
- `min_range`: 0.5
- `max_range`: 30.0
- `noise`: 0.008
- `min_angle`: -${M_PI}
- `max_angle`: ${M_PI}
- `lidar_link`: os1_lidar
- `imu_link`: os1_imu
- `vfov_min`: -.785
- `vfov_max`: .785

- Comments as of 01.27:
    - sensor is mounted upright for ease of viewing.
    - IMPORTANT: YP origin is at 0,0,-0.5 in sensor frame. This prevents scan from seeing "through" ship model.

### TODOs:
- [ ] Q: should lidar scans be in frame of sensor, or in frame of YP? 
    - A: SIM: sensor frame (i.e. cloud frame origin is where sensor is). If that's wrong, can later just compute 
        $$ T^{\text{yp}}_{\text{cloud}} = T^{\text{yp}}_{\text{sensor}} T^{\text{sensor}}_{\text{cloud}} $$
- [x] publish to set_model_state from python file
- [x] generate poses to publish (maneesh paper)
- [x] modify distribution to exclude "extreme" lidar angles
- [x] get ship regions as coordiantes  in the YP.stl frame\
- [x] algorithm to get class labels for each scan based on #points in each region in scan
- [x] get ship bounding boxes for each scan in sensor frame
- [ ] get psi to roll correctly

### Comments
#### PoseScatterGenerator.py
- If trying to plot (not publishing), use the conda base environment (requires uncommenting the block in the `~/.bashrc` file).
- If trying to write to a topic (not plotting), which uses `rospy`, it requires Python from `/usr/bin/python`. This does not have `pyvista` installed.

### Steps to sample
1. #### PoseScatterGenerator.py (in conda base env)
    - change the num_samples_C, and num_samples_F to the number of samples you want (must match)
    - change second value of r_C to be the maximum distance from the flight deck you want the sensor to go out to.
    - change the range of +- 1.5 to be the vertical pitch range allowed for the sensor
    ```        
    F_filtered = F[np.logical_and(F[:, 2] >= C[i][2] - 1.5, F[:, 2] <= C[i][2] + 1.5)]
    
    where first term (i.e. F[:, 2] >= C[i][2] - 1.5,) corresponds to downward pitch amount, and second term (i.e. F[:, 2] <= C[i][2] + 1.5) corresponds to upward pitch.
    ```
    - To check the settings before sampling, you can plot the points by runing ```$ python PoseScatterGenerator.py``` which also plotsa some generated CF vectors and poses.
    
2. Run ```$ roslaunch ouster_description os1_world.launch -v``` to start the gazebo simulation (make sure to source the workspace (on orin nano, ```/home/fdcl/Ouster/gazebo_ws_fdcl```)).
    - NOTE: this won't work if the conda base env is active... must be using the /usr/bin/python for this to work.
3. #### PoseServiceServer.py
    - Start the Server
4. #### PoseServiceClient.py
    - Begin the sampling. The first pose and scan will always start from the same place, regardless of where the sensor is in the simulation (i.e if you manually moved teh sensor beforehand). HOWEVER, if the ship model was moved, you need to restart the simulation since otherwise the ship and sensor origins wont' coincide.
