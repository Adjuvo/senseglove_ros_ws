# Senseglove ROS Workspace

A workspace for the integration of the SenseGlove into _ROS Noetic_.
This workspace makes use of ros_control for automatically initiating publisher and subscriber nodes for the state of the senseglove. If you face issues when dealing with sending haptic commands, make use of the *senseglove_haptics topic*

## SenseGlove Support Matrix

|             | **ROS Noetic** | **   ROS 2  ** |
|-------------|:--------------:|:--------------:|
| **DK1**     |   ✅ v1.0.0    |       ❌       |   
| **Nova 1**  |   ✅ v2.0.0    |       🔜       | 
| **Nova 2**  | ❗v2.0.0       |       🔜       | 
  

* <code>✅</code> Supported
* <code>❗</code> Not supported by the latest release and might be lacking features
* <code>❌</code> Not supported at all
* <code>❓</code> Unknown/untested

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

## Using sensegloves in ROS ##
The sensegloves are connected either through USB or Bluetooth, depending on the product. Senseglove Nova connects only through bluetooth, while DK1 connects only through USB.

The workspace also consists of the bash scripts for you to connect & disconnect your Nova device via bluetooth. Kindly navigate to senseglove->senseglove_launch->bluetooth_scripts to find these scripts.

For a detailed procedure on connecting a Nova or Nova 2 glove in linux, kindly refer to [SenseGlove Docs - Connecting Devices](https://senseglove.gitlab.io/SenseGloveDocs/connecting-devices.html), under Pairing SenseGlove Nova or Wireless Kit -> Linux.

**NOTE:** There are two launch files; one for dk, one for nova. The current implementation allows you to use either of these standalone gloves through these two launch files. Future updates can include simultaneous use of gloves.

### Example: Using two sensegloves [Left and Right] in ROS: ###
1. Source your workspace
2. Make sure your sensegloves are connected through usb or bluetooth
    - If you checked your connection with sensecom, be sure to exit the application before proceeding
3. In the `senseglove_demo.launch` script, make sure you specify the devices being used:
    - `use_dk`
    - `use_nova`
    - `use_nova2`

4. Finally, make sure you specify the handedness arguments:
    - `use_left` = true
    - `use_right` = true

5. Run: `roslaunch senseglove_launch senseglove.launch` after saving.

- A bash script is called invoking sensecom and running the hardware interface node twice for a left- and a right-handed glove.
- If all is well, your invocation of the roslaunch command should have started a roscore session and all necessary nodes providing intefaces to the senseglove.


### Example: Using a single senseglove in ROS: ###
Though the whole infrastructure of this codebase was built upon the use with infinitely many sensegloves, our example launch file only accepts two gloves.
Moreover, due to our integration into ros-control we require the user to know what type of gloves are connected to the PC.
As such, the user has to define which glove is connected to the system.

1. Find out if you are dealing with a left or right-handed senseglove.
2. In the `senseglove_demo.launch` script, make sure you specify the devices being used:
    - `use_dk`
    - `use_nova`
    - `use_nova2`

3. Finally and most importantly, give proper bool value to glove you are using:
    - `use_left` 
    - `use_right`

4. Run: `roslaunch senseglove_launch senseglove.launch` after saving.


## General Usage Description: ##
This repository is meant to present a solid foundation for using the senseglove in ROS Noetic. As such it provides no
concrete example projects. It does however provide the user with a few launch files, hardware.launch and senseglove_demo.launch
which initiate the sensegloves.
The senseglove_hardware_interface nodes which are called by these launch files do nothing more than using the senseglove API
in a ROS /ros_control "sanctioned" manner.

__Important:__ Run the sensecom application, present in the SenseGlove_API folder, when using hardware.launch! The launching of sensecom has been taken care of in *senseglove_demo.launch* in the bash script.

Users are advised to develop their own applications outside this package and make use of the provided topics. If users do find the need to 
write additions to this package, beware that this repository is still subject to changes and pulling this repo again might override your own code.

**If you, as a user, find a bug or have an issue with getting the workspace up and running, we suggest you leave this as an issue on this repository.**
This practice will allow others to troubleshoot their own problems quicker.

### Using the finger distance node: ###
The finger distance package is meant to publish the distance between the fingertips through a rosnode as a means to control robotic grippers. This package also provides a calibration class that provides a service server. The service is easily called from the rqt_service_caller plugin. Instructions for the calibration are printed on your terminal. This is currently implemented for only the DK1 gloves.