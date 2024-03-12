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
  * The ROS1_bridge package creates a docker container which hosts ROS1 Noetic, ROS2 Galactic, and the ROS1_bridge. The `state_node` and the `action_pick_node` are built and run from inside this container.  
* `actions\state`
  * The actions\state package is run directly on the ROBOTIS-OP3 robot. This package includes the `action_edit_node ` and the `action_play_node`.
* `noetic_audio`
  * The noetic_audio pakage creates a docker container which hosts ROS1 Noetic. This package includes the the `identify_bpm_node`.

## License and Copyright Information
The software included with ROBOTIS-OP3 comes is utilized in this project, some components modified and others unmodified. 
This sotware is licensed with Apache-2.0.
The orginial ROBOTIS-OP3 software can be found at the following: https://github.com/ROBOTIS-GIT/ROBOTIS-OP3.git.