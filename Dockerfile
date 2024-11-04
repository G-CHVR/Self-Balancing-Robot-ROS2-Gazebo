FROM ros:jazzy
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV ROS_DISTRO=jazzy

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL=/bin/bash

# Install Gazebo 
RUN sudo apt-get install -y ros-${ROS_DISTRO}-ros-gz

# Install tools and dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions

# Source the ROS 2 setup files after the workspace build
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/ws/install/setup.bash" >> /home/$USERNAME/.bashrc
USER $USERNAME
CMD ["/bin/bash"]
