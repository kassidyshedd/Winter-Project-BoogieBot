# First stage: ROS1 Noetic setup
FROM ros:noetic-ros-core-focal as ros1
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN echo "deb http://archive.ubuntu.com/ubuntu/ focal main universe" > /etc/apt/sources.list \
    && echo "deb http://archive.ubuntu.com/ubuntu/ focal-updates main universe" >> /etc/apt/sources.list \
    && echo "deb http://archive.ubuntu.com/ubuntu/ focal-security main universe" >> /etc/apt/sources.list \
    && apt-get update && apt-get install -y \
        build-essential \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-pip \
        libeigen3-dev \
        libyaml-cpp-dev \
        && rosdep init

# Create ROS1 workspace and install package dependencies
WORKDIR /root/ws_ros1
COPY ./state /root/ws_ros1/src/state

# Second stage: ROS2 Galactic setup
FROM ros:galactic-ros-base-focal as ros2
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Create ROS2 workspace
WORKDIR /root/ws_ros2
COPY ./messages /root/ws_ros2/src/messages
COPY ./action_pick /root/ws_ros2/src/action

# Third stage: Build ros1_bridge
FROM ros:galactic-ros-base-focal
SHELL ["/bin/bash", "-c"]

# Copy ROS1 and ROS2 workspaces
COPY --from=ros1 /root/ws_ros1 /root/ws_ros1
COPY --from=ros2 /root/ws_ros2 /root/ws_ros2

# Add the ROS Noetic source
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key F42ED6FBAB17C654

# Environment setup
RUN apt-get update && apt-get install -y \
    ros-galactic-ros1-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros1_bridge_ws

# Clone the galactic bridge repository
RUN mkdir -p src && \
    git clone -b galactic https://github.com/ros2/ros1_bridge src/ros1_bridge 



