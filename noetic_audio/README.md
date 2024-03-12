# noetic_audio package
The noetic_audio package contains two node used for tempo indentification. 
This package is intended to be set up as a docker container, containing all 
required elements. 

## ROS Nodes
* `identify_bpm_node`: Inputs a .wav file into shazamio's `recognoze_song` function, which outputs the song title and artist. This information is used by Spotipy's `search`
function to get the song's track_id. Finally, this track_id is put into Spotipy's `audio_feature` function, which returns the tempo of the song.

## How to use
* Navigate to the directory where the docker should be built and build.
  * `docker build -t <desired_container_name> $(pwd) `
* Run the docker
  * `docker run -it --network=host --name=<desired_docker_name> -v $(pwd):/catkin_ws/src  <desired_container_name>`
* Navigate to catkin_ws, build, source, etc..
* Launch audio node
  * `roslaunch audio identify_bpm`
