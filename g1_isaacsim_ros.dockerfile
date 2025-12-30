#. All rights reserved

# Use the official ROS 2 Humble base image
FROM ros:humble-ros-base

# Set environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-ur \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-rviz-visual-tools \
    ros-humble-rqt-tf-tree \
    ros-$ROS_DISTRO-rviz2 \
    python3-vcstool \
    libnlopt0 \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

ARG dUSER=ros2_nh
ARG HOME=/home/${dUSER}

# Create and set up the ROS 2 workspace
WORKDIR $HOME/ros2_ws/src

# User and permissions
ARG dGROUP=docker
ARG dUID
ARG dGID

# Create group, user, and set up sudo
RUN mkdir -p /etc/sudoers.d \
    && groupadd -g ${dGID} ${dGROUP} \
    && useradd -d ${HOME} -u ${dUID} -g ${dGID} -m -s /bin/bash ${dUSER} \
    && echo "${dUSER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/sudoers_${dUSER} \
    && chmod 0440 /etc/sudoers.d/sudoers_${dUSER}

# Modify user groups and set ownership as root
RUN usermod -a -G dialout ${dUSER} \
    && usermod -a -G video ${dUSER} \
    && chown -R ${dUSER}:${dGROUP} ${HOME}

# Switch to the user
USER ${dUSER}

# Build the workspace
WORKDIR $HOME/ros2_ws

# Isaac Sim 4.5.0
RUN pip install --ignore-installed -U sympy
RUN pip install torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cu126
RUN pip install --upgrade pip
RUN pip install 'isaacsim[all,extscache]==4.5.0' --extra-index-url https://pypi.nvidia.com
RUN pip install requests

# Source the workspace in the entrypoint
RUN echo "alias source_humble='source /opt/ros/$ROS_DISTRO/setup.bash'" >> $HOME/.bashrc
RUN echo "alias source_ws='source $HOME/ros2_ws/install/setup.bash'" >> $HOME/.bashrc

# Set the entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source $HOME/.bashrc && exec \"$@\"", "--"]

# Default command
CMD ["bash"]
