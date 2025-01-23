# syntax=docker/dockerfile:1
FROM ros:noetic-robot

# change the default shell to bash
SHELL ["/bin/bash", "-c"]

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    ros-noetic-roslint \
    python-is-python3 \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install yq from GitHub
RUN wget -qO /usr/bin/yq https://github.com/mikefarah/yq/releases/latest/download/yq_linux_amd64 && \
    chmod +x /usr/bin/yq

# Create the catkin workspace and set working directory
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Copy the config_input.yaml file
COPY /config-data/config_input.yaml /config-data/config_input.yaml

# Create and prepare the cloning script to install ROS packages
COPY clone_repos.sh /root/catkin_ws/src/clone_repos.sh

# Run the cloning script
RUN /root/catkin_ws/src/clone_repos.sh

# Build workspace after cloning
WORKDIR /root/catkin_ws
RUN source /ros_entrypoint.sh && catkin_make && source devel/setup.bash

# Copy Python script to get config
COPY get_config.py /root/get_config.py

# Default command
CMD ["/bin/bash", "-c", "source /ros_entrypoint.sh && \
  roscore & sleep 5 && \
  source /root/catkin_ws/devel/setup.bash && \
  python3 /root/get_config.py"]
