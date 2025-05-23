FROM ghcr.io/selkies-project/nvidia-egl-desktop:24.04-20241222100454

SHELL ["/bin/bash", "-c"]

# Disable terminal interactivity
ENV DEBIAN_FRONTEND=noninteractive

# Install some essential packages
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    bash-completion \
    command-not-found \
    software-properties-common \
    xdg-user-dirs \
    wget \
    build-essential \
    cmake \
    gdb \
    git \
    openssh-client \
    python3-argcomplete \
    python3-pip \
    && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Install ROS2 Jazzy
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        ros-jazzy-desktop \
        ros-dev-tools \
        ros-jazzy-ament-* \
        && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Install colcon and rosdep
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Create working directory
WORKDIR /home/ubuntu/ros2_ws/src

# Copy the project files into the working directory
COPY . /home/ubuntu/ros2_ws/src/

# Copy workspace configuration script
COPY workspace.sh /home/ubuntu/

# Make script executable
RUN sudo chmod +x /home/ubuntu/workspace.sh

# Run the workspace setup script to install dependencies and build the ROS2 packages
RUN cd /home/ubuntu && bash ./workspace.sh

# Install Gazebo Harmonic
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && sudo apt-get update \
    && sudo apt-get install ros-jazzy-ros-gz -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Remove workspace script
RUN rm workspace.sh
