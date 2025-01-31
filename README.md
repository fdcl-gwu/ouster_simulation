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
    - YP origin is at 0,0,-0.5 in sensor frame. This prevents scan from seeing "through" ship model.

### TODOs:
- [ ] Q: should lidar scans be in frame of sensor, or in frame of YP? 
    - A: SIM: sensor frame (i.e. cloud frame origin is where sensor is). If that's wrong, can later just compute 
        $$ T^{\text{yp}}_{\text{cloud}} = T^{\text{yp}}_{\text{sensor}} T^{\text{sensor}}_{\text{cloud}} $$
- [x] publish to set_model_state from python file
- [x] generate poses to publish (maneesh paper)
- [x] modify distribution to exclude "extreme" lidar angles
- [ ] get ship bounding boxes for each scan in sensor frame
- [ ] get psi to roll correctly

### Comments
#### pose_algorithm.py
- If trying to plot (not publishing), use the conda base environment (requires uncommenting the block in the `~/.bashrc` file).
- If trying to write to a topic (not plotting), which uses `rospy`, it requires Python from `/usr/bin/python`. This does not have `pyvista` installed.