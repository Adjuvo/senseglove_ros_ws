# Senseglove ros_workspace

A workspace for the integration of the SenseGlove into ROS Melodic.
This workspace makes use of ros_control for automatically initiating publisher and subscriber nodes for the state of the senseglove.

## General Usage: ##

This repository is meant to present a solid foundation for using the senseglove in ROS Melodic. As such it provides no
concrete example projects. It does however provide the user with a few launch files, hardware.launch and senseglove_demo.launch 
which initiate the sensegloves.
The senseglove_hardware_interface nodes which are called by these launch files do nothing more than using the senseglove API
in a ROS /ros_control "sanctioned" manner.

__Important:__ Run the sensecom application, present in the SenseGlove_API folder, before launch!

Users are advised to build their own nodes in the senseglove folder as if it were a separate repository when transforming 
values from the senseglove. When making use of the topics in your own application, it is expected that these nodes are developed 
in your own package.

## To do: ##
* Implement standard CLang format, currently none are used (code adheres to no singular style, big oof)
* Custom Exceptions for easy debugging
* adjust .gitlab-ci.yml to include clang only if it does not check the SenseGlove_API package due to style differences.