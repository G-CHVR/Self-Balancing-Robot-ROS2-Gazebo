FROM ros:humble
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

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

RUN sudo apt-get update && sudo apt-get install -y lsb-release curl gnupg

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN sudo apt-get update && sudo apt-get install -y gz-garden 

# Install tools and dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-plotjuggler-ros \
    ros-humble-rqt \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

COPY requirements.txt /home/ws/requirements.txt
RUN pip install -r /home/ws/requirements.txt

# [Optional] Set the default user. Omit if you want to keep the default as root.

# Source the ROS 2 setup files after the workspace build
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/ws/install/setup.bash" >> /home/$USERNAME/.bashrc
USER $USERNAME
CMD ["/bin/bash"]
