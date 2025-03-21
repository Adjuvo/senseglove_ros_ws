# Senseglove ROS Workspace

A workspace for the integration of the SenseGlove into _ROS Noetic_.
This workspace makes use of ros_control for automatically initiating publisher and subscriber nodes for the state of the senseglove.

## SenseGlove Support Matrix

|             | **ROS Noetic** | **   ROS 2  ** |
|-------------|:--------------:|:--------------:|
| **DK1**     |   ✅ v1.0.0    |       ❌       |   
| **Nova 1**  |   ✅           |       🔜       | 
| **Nova 2**  |   ✅           |       🔜       | 
  

* <code>✅</code> Supported
* <code>❗</code> Not supported by the latest release and might be lacking features
* <code>❌</code> Not supported at all
* <code>❓</code> Unknown/untested

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

## Setting up the workspace ##
1. System Requirement: Ubuntu 20.04
2. Install [ros-noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
3. Clone our workspace
``` 
git clone https://github.com/Adjuvo/senseglove_ros_ws.git
``` 
4. Install the following dependencies & update:
``` 
sudo apt-get install ros-noetic-ros-control
sudo apt-get install ros-noetic-joint-trajectory-controller
sudo apt-get update
rosdep update
sudo apt-get upgrade
``` 
5. Navigate to the workspace folder in the terminal
6. Make sure to source your workspace
7. Build your workspace: `catkin build` or `catkin_make`
8. Post building, source the workspace itself using `source devel/setup.bash`

## Using SenseGloves in ROS ##
- The sensegloves are connected either through USB or Bluetooth, depending on the product. 
- The `Nova (1 & 2)`  has bluetooth communication.
- The `DK1` has USB communication.

#### Connecting NOVA Gloves via Bluetooth ####

- The workspace consists of the bash scripts to connect & disconnect your Nova device via bluetooth. 

```
rosrun senseglove_launch glove_connect.sh glove_connect.sh
rosrun senseglove_launch glove_connect.sh glove_disconnect.sh
```

- For a detailed procedure on connecting a Nova glove(1 & 2)in linux, kindly refer to [SenseGlove Docs - Connecting Devices](https://senseglove.gitlab.io/SenseGloveDocs/connecting-devices.html), under Pairing SenseGlove Nova or Wireless Kit -> Linux.


#### `NOTE` ####
- The main launch file: `senseglove.launch` has args to specify which standalone glove you are about to use. The current implementation allows you to use either of these standalone gloves. Future updates can include simultaneous use of gloves.
- Calibration of the Nova-2 device runs on the glove itself. The SenseCom software offers a visual guide to accompany calibration. For more information, go to [SenseGlove Docs/Nova-2](https://senseglove.gitlab.io/SenseGloveDocs/nova-2.html)

---

### Example: Using two SenseGloves [Left and Right]: ###
1. Source your workspace
2. Make sure your sensegloves are connected through usb or bluetooth
    - If you are checking your connection by running a sensecom instance, be sure to exit the application before proceeding to avoid runing multiple instances of sensecom during launch.

3. In the `senseglove.launch` script, make sure you specify the devices being used:
    - `use_dk`
    - `use_nova`
    - `use_nova2`

4. Finally, make sure you specify the handedness arguments:
    - `use_left` = true
    - `use_right` = true

5. Run: 
```
roslaunch senseglove_launch senseglove.launch
```

#### `NOTE` ####
- A bash script is called invoking sensecom and running the hardware interface node twice for a left- and a right-handed glove.
- The bash **`waits for the user input`** to confirm whether the gloves are connected in SenseCom. 

---

### Example: Using a single SenseGlove [Left or Right]: ###
- Though the whole infrastructure of this codebase was built upon the use with infinitely many sensegloves, our example launch file only accepts two gloves.
- Moreover, due to our integration into ros-control we require the user to know what type of gloves are connected to the PC.
- As such, the user has to define which glove is connected to the system.

1. Find out if you are dealing with a left or right-handed senseglove.
2. In the `senseglove.launch` script, make sure you specify the devices being used:
    - `use_dk`
    - `use_nova`
    - `use_nova2`

3. Finally and most importantly, give proper bool value to glove you are using:
    - `use_left` 
    - `use_right`

4. Run: 
```
roslaunch senseglove_launch senseglove.launch
```

## Finger-Tip Distances: ##
The finger distance package is designed to publish the distances between fingertips via a ROS node, which can be used to control robotic humanoid hands and grippers.
- **Calibration Service**: A calibration class is provided as a service server. This service can be easily called using the rqt_service_caller plugin, or through the terminal.
```
rosservice call /senseglove_finger_distance_left/Calibrate left
rosservice call /senseglove_finger_distance_right/Calibrate right
```
 
- A Calibration Gui will appear. Install PyQt5 if it is not already available.
```
sudo apt-get install python3-pyqt5
```

- You can find the defaults/calibrated parameters in the [calibration folder](src/senseglove/senseglove_shared_resources/calibration/).

## Haptics: ##
- ROS-Control for the force-feedback system. Refer to the joints & controllers assigned for each senseglove product in the [config folder](/src/hardware_interface/senseglove_hardware_interface/config/).
- Refer to the python scripts for haptic implementation in the [senseglove_haptics folder](/src/senseglove/senseglove_haptics/src/senseglove_haptics/).
- `NOTE for NOVA 2`: The vibration feedback is disabled for the Index,  Thumb and the palm locations because of a packet overloading issue. A custom_waveform service will be implemented instead of employing ros-control.

## TO-DO: ##
- `Custom_waveform` service for Nova-2 vibrations
- `Calibration Profiling`, either from SenseCom (or) as a service call with interactive GUI. This should allow us to access and control the calbration.
- `IMU` access (incl. sensor value if possible)