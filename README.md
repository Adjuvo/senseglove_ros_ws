# Senseglove ros_workspace
A workspace for the integration of the SenseGlove into _ROS Melodic_.
This workspace makes use of ros_control for automatically initiating publisher and subscriber nodes for the state of the senseglove.

Florent Audonnet has done some work on support for ROS2 Galactic. In case you are interested, have a look at https://github.com/09ubberboy90/senseglove_ros_ws/tree/ros2

If the current build (after the visualization branch merge) breaks your current work environment, please refer to [this older version]: https://github.com/Adjuvo/senseglove_ros_ws/commit/f0126b165fc865e1ce0be19db2f68bf725c221da of this workspace. 


## 1. For ROS beginners: ##
If you are totally unfamiliar with ROS we advise you to take a look at the ROS-wiki for a quick startup guide.
We especially recommend the following tutorials:
* http://wiki.ros.org/melodic/Installation/Ubuntu
* http://wiki.ros.org/ROS/StartGuide
* http://wiki.ros.org/ROS/Tutorials
    * http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
    * http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

A large part of understanding/working with ROS is the ability to search for the issues you are dealing with.
In our experience, simply googling for an error or issue you have can do the trick. Do keep in mind that there exists a
vast amount of documentation, though it can be tedious to sort through its information.

### Setting up the workspace for the senseglove: ###
1. Make sure you are on a Ubuntu 18.04 system (not necessary, but all code has been checked with docker images running in ubuntu).
2. Download ros-melodic as described in the ros-wiki.
3. clone our workspace by either using `git clone https://github.com/Adjuvo/senseglove_ros_ws.git` or through the webbrowser here on the Github website.
4. install the following dependencies:
    1. `sudo apt-get install ros-melodic-ros-control`
    2. `sudo apt-get install ros-melodic-joint-trajectory-controller`
5. Run: `sudo apt-get update`
6. Run: `rosdep update`
7. Run: `sudo apt-get upgrade`
8. navigate, in the terminal, to the workspace folder
9. source your workspace: `source /opt/ros/melodic/setup.bash`
10. Build your workspace: `catkin build`
    1. or if you prefer a different build tool use: `catkin_make`
    2. or use: `colcon build`
11. after building, you can source the workspace itself by using `source devel/setup.bash`

## 2. General Usage: ##
This repository is meant to present a solid foundation for using the senseglove in ROS Melodic. As such it provides no
concrete example projects. It does however provide the user with a few launch files, hardware.launch and senseglove_demo.launch
which initiate the sensegloves.
The senseglove_hardware_interface nodes which are called by these launch files do nothing more than using the senseglove API
in a ROS /ros_control "sanctioned" manner.

__Important:__ Run the sensecom application, present in the SenseGlove_API folder, when using hardware.launch! The launching of sensecom
has been taken care of in senseglove_demo.launch in the bash script.

Users are advised to develop their own applications outside this package and make use of the provided topics. If users do find the need to 
write additions to this package, beware that this repository is still subject to changes and pulling this repo again might override your own code.

**If you, as a user, find a bug or have an issue with getting the workspace up and running, we suggest you leave this as an issue on this repository.**
This practice will allow others to troubleshoot their own problems quicker.

### Example; using two sensegloves in ROS: ###
1. source your workspace
2. make sure your sensegloves are connected through usb or bluetooth
    1. if you checked your connection with sensecom, be sure to exit the application before proceeding
3. run: `roslaunch senseglove_demo.launch`

A bash script is called invoking sensecom and running the hardware interface node twice for a left- and a right-handed glove.
If all is well, your invocation of the roslaunch command should have started a roscore session and all necessary nodes providing intefaces to the senseglove.
In a second (properly sourced) terminal you can verify that these nodes are publishing by invoking: rostopic list
you can further test the application by checking that these topics get published by invoking: rostopic echo /topic_name

### Example; using a single senseglove in ROS: ###
Though the whole infrastructure of this codebase was built upon the use with infinitely many sensgloves, our example launch file only accepts two gloves.
Moreover, due to our integration into ros-control we require the user to know what type of gloves are connected to the PC.
As such, the user has to define which glove is connected to the system.

1. find out if you are dealing with a left- or right-handed senseglove
2. remove the non-existing glove from the senseglove_hardware_demo.launch file, such that only your left/right-handed glove remains.
3. save the launch file
4. build you workspace
5. source your workspace
6. proceed as if you were dealing with 2 sensegloves

### Remarks for using the finger distance node: ###
The finger distance package is meant to publish the distance between the fingertips through a rosnode as a means to control robotic grippers. This package also provides a calibration class that provides a service server. The service is easily called from the rqt_service_caller plugin.Instructions for the calibration are printed on your terminal.
## 3. To do: ##
This is a small to do list for the upcoming features in this repository these will be added as issues as well.
* Custom Exceptions for easy debugging
* Provide speed and acceleration data of the fingertippositions as well as for the encoder data.
