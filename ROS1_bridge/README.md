# ROS1_bridge Package
The ROS1_bridge package contains two nodes and the ROS1-BRIDGE (https://github.com/ros2/ros1_bridge). 
This package is intended to be set up as a docker container, containing all required elements. 

This container is built using a multi-stage Dockerfile. The first stage builds a ROS1 Noetic 
environment, the second stage builds a ROS2 Galactic environment, and the final stage builds
the ROS1_bridge environment (ROS2 Galactic base). 

## ROS Nodes
* `state_node`: Handles all state changes required by the project.
* `action_pick_node`: Randomly picks actions that best correspond witht the song's tempo.
* `dynamic_bridge`: From ROS1-BRIDGE, creates bridges for all topics.

## ROS1_bridge custom message
* ROS1 Noetic message -> (state::msg::RandomList)
* ROS2 Galactic message -> (messages::msg::RandomList)

## How to use
Navigate to the directory where the docker should be built and build
* `docker build -t <desired_container_name> $(pwd) `
Run the docker
* `docker run -it --network=host --name=<desired_docker_name> -v $(pwd):/catkin_ws/src  <desired_container_name>`
Open 3 total instances of the docker container
  * `docker exec -it <desired_docker_name /bin/bash` (2x)
<br>
In the first instance of the container
* `export ROS_DISTRO=`
Source ROS1 install
* `source /opt/ros/noetic/setup.bash`
Navigate to ws_ros1 & build
* `catkin_make_isolated --install`
Source, build, and launch state_node
* `roslaunch state state_node`
<br>
* In the second instance of the container
  * `export ROS_DISTRO=`
* Source ROS2 install, navigate to ws_ros2, and build
* Source build and launch action_pick_node
  * `roslaunch action_pick action_pick_node`
<br>
* In the third instance of the container (Follow Exactly)
  * `export ROS_DISTRO=`
  * Source ROS1 install
  * Source ROS2 install
  * Source ROS1 build
    * `source ws_ros1/install_isolated/setup.bash`
  * Source ROS2 build
    * `source ws_ros2/install/local_setup.bash`
  * Navigate to ros1_bridge directory and build
    * `colcon build --cmake-force-configure`
  * Source ROS1_bridge build
    * `source install/local_setup.bash`
  * Run the bridge
    * `ros2 run ros1_bridge dynamic_bridge --bridge-all-topics`

  

