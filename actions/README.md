# Actions/state ROS1 Kinetic Package
The actions/state package contains two nodes used in the dynamic dance routine generation. This package should be run directly on the ROBOTIS-OP3 operating system. 


## ROS Nodes
* `action_edit_node`: Creates a YAML action file of all actions that will be used in 
  the dance routine. 
* `action_play_node`: Executes the actions, creating a dance sequence. This node utilizes the original ROBOTIS_OP3 software. 
* `op3_manager`: Initializes the robot. This is an unmodified node from the ROBOTIS-OP3 software
* `ros_madplay_player`: Plays audio from ROS node.


## How to use
To launch all nodes required for proper initialization of the robot and action nodes:
`roslaunch state action_nodes.xml`

Use a service call to start the process:
`rosservice call /listen_trigger`
