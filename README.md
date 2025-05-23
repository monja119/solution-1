# PARC2025-Engineers-League
Pan-African Robotics Competition (PARC) Engineer's League 2025 project development.

## Package Overview

- [`parc_robot_bringup`](./parc_robot_bringup/) : Contains config, world, scripts and launch files to bringup the PARC AgRobot for the autonomy task.
- [`parc_robot_description`](./parc_robot_description/) : Contains the URDF description files for the PARC AgRobot and launch files for the robot state publisher and description.

## Docker Usage

The Ubuntu environment and ROS2 workspace and packages are also available in a containerized environment using docker. The following subsections details the process of setting this environment. This was tested on an Ubuntu 24.04 computer, compatibility with Windows has yet to be confirmed.

<!-- Make more general for windows PCs later on in the docs -->

### Install Nvidia Container ToolKit

This is required to enable use of the NVIDIA GPU in the docker container. Installing the NVIDIA GPU driver on your Linux distribution is a [prerequisite](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#prerequisites) before running the following commands.

Configure the production repository:
```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Update the packages list:

```
sudo apt-get update
```

Install the NVIDIA Container Toolkit packages:

```
sudo apt-get install -y nvidia-container-toolkit
```

### Configuring Docker

The container runtime is configured using the `nvidia-ctk` command:
```
sudo nvidia-ctk runtime configure --runtime=docker
```

Restart the Docker daemon for the changes to be effected:
```
sudo systemctl restart docker
```

### Pull the project image

Run the following command to pull the project docker image:

```
docker pull thenoobinventor/parc-eng-2025-ws:latest
```

### Run the container

Before running the container, there are a few things that need to be sorted out. First of all, setup the ROS workspace on your computer; choose a name for a workspace if `ros2_ws` is already in use.

Open a new terminal on your PC, then copy and paste the following one line at a time: 

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Next clone this project repository:

```
cd ~/ros2_ws/src
git clone https://github.com/PARC-Robotics/PARC2025-Engineers-League.git .
```

The container will be started using `docker compose`. The following changes will need to be made to the [docker-compose.yml](docker-compose.yml) file. Under the `volumes` section,

```
volumes:
- '/home/noobinventor/ros2_ws/src:/home/ubuntu/ros2_ws/src'
```

replace `noobinventor/ros2_ws` with your PC username and the chosen workspace name on your PC, if it differs from `ros2_ws`, then save these changes.

While still in the `src` directory, run this command to start the docker container:

```
docker compose up -d
```

Open a browser tab and go to this address: `127.0.0.1:8080`. You will be prompted for login credentials which are

```
username: ubuntu
password: mypasswd
```

The image below is obtained from running the autonomy track launch command: 

```
ros2 launch parc_robot_bringup task_launch.py
```

<p align="center">
  <img title='autonomy track view' src=docs/images/autonomy_track_view.png width="800">
</p>

To stop and remove the container run:

```
docker compose down
```

The volume setup in the `docker-compose.yml` file for the workspace `src` directory ensures that files in the directory persist after the container is stopped.

The full documentation for the 2025 PARC Engineer's League is available [here](https://parc-robotics.github.io/documentation-2025/introduction/).


## Reference
[Virtual Maize Field](https://github.com/FieldRobotEvent/virtual_maize_field)