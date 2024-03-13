# Copyright 2024 - Andrew Kwok Fai LUI, Robotics and Autonomous System Group,
# Research Engineering Facility, RI
# the Queensland University of Technology
ARG ROS_DISTRO=noetic

FROM osrf/ros:${ROS_DISTRO}-desktop-full
LABEL author "Andrew Lui <luia2@qut.edu.au>"

ARG USER=qcr
ENV USER=${USER}
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ARM_WS=/home/${USER}/arm_commander_ws

# update apt-get and add user 'qcr' grant sudo privilege
RUN apt-get update && \
    apt-get install -y sudo && \
    adduser --disabled-password --gecos "" qcr  && \
    echo "${USER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER ${USER}
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]

# complete rosdep installation
RUN rosdep update

# install base packages
RUN sudo apt-get update && sudo apt-get install -y \
lsb-release \
git \
openssh-server \
ca-certificates \
net-tools \
vim

RUN git config --global core.editor "vim"

RUN sudo apt-get install -y \
locales \
wget \
curl \
dbus \
htop \
figlet \
build-essential \
cmake \
gdb

# install python
RUN sudo apt-get install -y \
python3-pip \
python3-rosdep \
python3-setuptools

# install jypyter-ros for running ros in jupyter notebook
RUN pip install numpy==1.24.4 notebook==6.5.5 ipykernel py-trees==2.2.3

# install moveit
RUN sudo apt-get install ros-${ROS_DISTRO}-moveit -y
# install UR10
RUN sudo apt-get install ros-${ROS_DISTRO}-ur10-moveit-config -y
# install arm commander
RUN mkdir -p ${ARM_WS}/src
RUN cd ${ARM_WS}/src && \
    git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel
RUN cd ${ARM_WS}/src && \
    git clone https://github.com/REF-RAS/arm_commander
RUN cd ${ARM_WS}/src && \
    git clone https://github.com/REF-RAS/task_trees    

RUN cd ${ARM_WS} && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${ARM_WS} && \
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# set locales
RUN sudo locale-gen en_US.UTF-8  
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8

# modify the login script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
CMD ["bash"]

