# syntax=docker/dockerfile:1
FROM ros:noetic-robot

# change the default shell to bash
SHELL ["/bin/bash", "-c"]

# set up environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

# create the catkin workspace and set working directory
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# install necessary dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# clone the necessary repositories
WORKDIR /root/catkin_ws/src

# Copy the repos.txt file
COPY /config-data/repos.txt /config-data/repos.txt

# Create and prepare the cloning script
RUN printf '#!/bin/bash\nset -e\nwhile read -r repo; do\n  git clone "$repo"\ndone < /config-data/repos.txt\n' > /root/catkin_ws/src/clone_repos.sh && \
    chmod +x /root/catkin_ws/src/clone_repos.sh

# Run the cloning script
WORKDIR /root/catkin_ws/src
RUN /root/catkin_ws/src/clone_repos.sh

# build workspace after cloning
WORKDIR /root/catkin_ws
RUN source /ros_entrypoint.sh && catkin_make && source devel/setup.bash

COPY get_config.py /root/get_config.py
COPY /config-data/requirements.txt /root/requirements.txt


# Default command
CMD ["/bin/bash", "-c", "pip install -r /root/requirements.txt && \
  source /ros_entrypoint.sh && \
  roscore & sleep 5 && \
  source /root/catkin_ws/devel/setup.bash && \
  python3 /root/get_config.py"]
