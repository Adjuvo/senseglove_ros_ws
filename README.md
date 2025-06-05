# Senseglove ROS Workspace

A workspace for the integration of the SenseGlove into _ROS Noetic_.
This workspace makes use of ros_control for automatically initiating publisher and subscriber nodes for the state of the senseglove.

## SenseGlove Support Matrix


|             | **ROS Noetic** |    **ROS 2**   |
|-------------|:--------------:|:--------------:|
| **DK1**     |   ✅ v1.0.0    |   🔜 Planned   |   
| **Nova 1**  |   ✅           |   🔜 Planned   | 
| **Nova 2**  |   ✅           |   🔜 Planned   | 

<code>✅</code> Supported
<code>🔜</code> In Progress
<code>❌</code> Not supported at all


## Directory Structure
    senseglove_ros
    |    ├── senseglove      
    |    |    ├── senseglove_control
    |    |    |    ├── senseglove_hardware            # Communicates w/ SenseGlove Hardware
    |    |    |    ├── senseglove_hardware_builder    # Builds the device in ROS
    |    |    |    ├── senseglove_hardware_interface  # The bridge b/w ros_control & SenseGlove hardware
    |    |    ├── senseglove_description              # Provides the URDFs & .rviz files
    |    |    ├── senseglove_interaction              # Provides the python scripts for finger distances, haptics, and other possible interactions
    |    |    ├── senseglove_launch                   # Launch files & Bluetooth scripts
    |    |    |   ├── bluetooth_scripts
    |    |    ├── senseglove_shared_resources         # Custom messages, services
    |    ├── SenseGlove_API                           # SG-Backend

## Installation ##      
1. Install [ros-noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on Ubuntu 20.04
2. Clone the repository: 
``` 
git clone https://github.com/Adjuvo/senseglove_ros.git
``` 
3. Install workspace dependencies: 
``` 
sudo apt-get install ros-noetic-ros-control \
                     ros-noetic-joint-trajectory-controller \
                     python3-pyqt5
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get upgrade
``` 
4. Build the workspace: `catkin build` or `catkin_make`
``` 
catkin_make
``` 
5. Source the workspace:
``` 
source devel/setup.bash
```

## Usage 
See [USAGE.md](USAGE.md) for instructions on connecting senseglove devices, launching single or dual glove setups, running calibration services, and enabling haptic feedback.

## Docker Setup [ROS1 -> ROS2]
See [DOCKER.md](Dockerfiles/DOCKER.md) for complete setup instructions on running the ROS 1 Noetic container with access to /dev/rfcomm*, allowing Bluetooth-based Nova 1/2 gloves to be used inside the container. The guide also covers how to optionally launch a ROS 1 to ROS 2 bridge so that ROS topics and services from the container can be accessed within a ROS 2 Jazzy environment, either on the host machine or in a connected container.

## TO-DO: ##
- `Custom_waveform` service for Nova-2 vibrations
- `Calibration Profiling`, either from SenseCom (or) as a service call with interactive GUI. This should allow us to access and control the calbration.
- `IMU_TF_Broadcaster`, the current implementation does not have the right tf conversion.

## Attribution

This project’s Dockerfile [Dockerfile.ros2_bridge](Dockerfiles/Dockerfile.ros2_bridge) and build instructions are adapted from [ros-jazzy-ros1-bridge-builder](https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder) by TommyChangUMD, licensed under MIT.