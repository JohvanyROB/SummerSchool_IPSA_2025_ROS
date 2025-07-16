#!/bin/bash

sudo apt update -y

# Install project's dependencies 
sudo apt -y install python3-colcon-common-extensions python3-pip
sudo apt -y install cppzmq-dev  #For ROS Development Studio

source /opt/ros/jazzy/setup.bash
sudo apt -y install ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-tf-transformations

source /opt/ros/jazzy/setup.bash
export PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated" #For ROS Development Studio
cd ~/ros2_ws && colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
