# Use ros:noetic as base image
FROM ros:noetic

# Set working directory
WORKDIR /root

# Copy contents from directory into docker
COPY . .

# Update package list
RUN apt-get update -y

# Install pip
RUN apt-get install -y python3-pip

RUN apt-get install -y ffmpeg

# Install dependencies
RUN pip3 install shazamio
RUN pip3 install spotipy
RUN pip3 install sounddevice
RUN pip3 install soundfile
RUN apt-get install -y apt-utils
RUN apt-get install -y portaudio19-dev

# Install text editor
RUN apt-get install -y nano


