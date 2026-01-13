#!/bin/bash

# ROS 2 Control and Hardware Interface packages
sudo apt install -y \
  ros-humble-hardware-interface \
  ros-humble-controller-interface \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster

# MoveIt 2 packages
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-planners \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-setup-assistant

# Robot description and visualization
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins

# TF and warehouse
sudo apt install -y \
  ros-humble-tf2-ros \
  ros-humble-warehouse-ros-mongo

# Serial communication
sudo apt install -y \
  libserial-dev

echo "All dependencies installed successfully!"