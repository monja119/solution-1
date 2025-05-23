#!/bin/bash
set -e

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash

# Navigate back to the workspace
cd ~/ros2_ws

# Update system repositories
sudo apt-get update

# Initialize rosdep and install dependencies
rosdep init
rosdep update
sudo rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y

# Compile the project packages
colcon build

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Source the .bashrc file to effect changes made to it when opening a new terminal
source ~/.bashrc