# ros_workspace

A workspace for the integration of the SenseGlove into ROS Melodic. 

DONE:
* Structured the code/files so that there is a clear distinction between the interface with the hardware, the interface with ros and the build environment.
* Implemented some higher level "pseudo-"code that will structure following code. 
* Implemented SenseGlove-API GitHub repo
* implement CI (Travis or Gitlab proprietary) to ensure clean build environments

TO DO:
* Implement standard CLang format, currently none are used (code adheres to no singular style (big oof))
* Implement the SenseGlove communication methods in sense_com.cpp
* Implement SenseGlove methods in the read, write function in the HWI
* Structure the urdf/xacro file(s)
* Custom Exceptions for easy debugging
* CMakeLists.txt --> transition from hardcode path to include directory using find_path()
* adjust .gitlab-ci.yml to include clang only if it does not check the SenseGlove_API package due to style differences.
* Give each SenseGlove a unique HWI so that multiple/varying controllers can be used at runtime