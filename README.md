# Dynamic Dance Routine Generation with ROBOTIS-OP3 Humanoid Robot
The ROBOTIS-OP3 Humanoid robot generates dynamic dance routines. This system analyzes music files 
to choreograph dance sequences that align with the rhythm of the music. Leveraging technologies 
such as shazamio, Spotipy, and a multi-docker framework. 


## Core Features
* Music Analysis: Utilizes shazamio to identify songs and Spotipy to determine the song's tempo.
* Dyanmic Choreography: Generates dace sequences based on the song's tempo and the robot's capabilities.
* ROS Integration: Employs a multi-docker framework to allow for communication between ROS1 and ROS2 environments. 
* State Machine Design: Implements a state machone for managing the robot's operational stage. 

## Dependencies
  * Docker
  * shazamio
  * Spotipy
  
## Packages
* `ROS1_bridge`
  * The ROS1_bridge package creates a docker container which hosts ROS1 Noetic, ROS2 galactic, and the ROS1_bridge. The `state_node` and the `action_pick_node` are built and run from inside this container.  
* `actions\state`
* `noetic_audio`