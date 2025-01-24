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

### Modification for OS0-32 rev0 specifications: