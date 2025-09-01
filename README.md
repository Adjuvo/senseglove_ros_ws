# Senseglove ROS Workspace

A workspace for the integration of the SenseGlove into _ROS2 Jazzy_.

🌱 The ROS 2 jazzy support is not complete. The [To-Do]() list at the bottom will update the plan for the coming weeks

## Directory Structure
    senseglove_ros
    |    ├── senseglove
    |    |    ├── senseglove_bringup                  # Launch files & Bluetooth scripts    
    |    |    ├── senseglove_control
    |    |    |    ├── senseglove_hardware            # SenseGloveRobot
    |    |    |    ├── senseglove_hardware_builder    # HardwareBuilder
    |    |    |    ├── senseglove_hardware_interface  # Hardware Interface
    |    |    ├── senseglove_description              # URDFs
    |    |    ├── senseglove_msgs                     # SenseGlove messages, services
    |    |    ├── senseglove_interaction              # Python Interaction Package
    |    ├── senseglove_api                           # SG-Backend
    |    ├── senseglove_com                           # SenseCom

## Installation ##      
1. Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) on Ubuntu Noble 24.04
2. Clone the repository: 
``` 
git clone -b ${ROS_DISTRO} https://github.com/Adjuvo/senseglove_ros.git
``` 
3. Install workspace dependencies: 
``` 
sudo apt-get update
sudo apt-get install ros-jazzy-ros2-control \
                     ros-jazzy-ros2-controllers \
                     python3-pyqt5


rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get upgrade
``` 
4. Build the workspace and source:
``` 
colcon build --symlink-install
source install/setup.bash
``` 

## Usage 
See [USAGE.md](USAGE.md) for instructions on connecting senseglove devices, launching single or dual glove setups, running calibration services, and enabling haptic feedback.

## Docker Setup [ROS1 ⇄ ROS2]
See [DOCKER.md](Dockerfiles/DOCKER.md) for complete setup instructions on running the ROS 1 Noetic container with access to /dev/rfcomm*, allowing Bluetooth-based Nova 1/2 gloves to be used inside the container. The guide also covers how to launch a ROS 1 to ROS 2 bridge so that ROS topics and services from the container can be accessed within a ROS 2 Jazzy environment, either on the host machine or in a connected container.

## ☐☑ TO-DO:  ##
- ☐ `Docker:` Update setup for ROS2
- ☐ `SenseGloveStates:` A broadcaster for gloves states such as finger-tip postion, IMU data etc.
- ☐ `Finger-Tip Distance:` Update Calibration and finger-tip distance node, add separate launch
-  ☐ `Vibration Haptic Controller:` Separate controller to input custom vibration for Nova-2
- `Calibration Profiling:` Allow to use the calibration profile from SenseCom