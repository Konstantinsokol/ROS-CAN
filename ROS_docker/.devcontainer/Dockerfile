FROM osrf/ros:noetic-desktop

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install pip
RUN sudo apt install -y python3-pip

# Install serial lib
RUN sudo apt install -y python3-serial

# Install Git
RUN sudo apt install -y git

# Install can-utils for CAN
RUN sudo apt install -y can-utils

#Install network linux tools
RUN sudo apt install -y net-tools

#Install kernel modules command
RUN sudo apt install -y kmod

#CAN modules
RUN sudo modprobe can_dev && sudo modprobe can && sudo modprobe can_raw && sudo modprobe vcan

#CANdo drivers
RUN sudo apt install -y libusb-1.0.0-dev

#To create the docker group and add your user: (for vscode(permition docker run))
#RUN sudo groupadd docker
#RUN sudo usermod -aG docker $USER

#IP command
#RUN sudo apt install -y iproute2

#CANdo drivers


# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

