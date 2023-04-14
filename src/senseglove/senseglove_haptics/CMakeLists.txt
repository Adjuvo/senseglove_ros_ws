cmake_minimum_required(VERSION 3.0.2)
project(senseglove_haptics)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    genmsg
    rospy
    senseglove_shared_resources
    std_msgs
    control_msgs
    trajectory_msgs
    dynamic_reconfigure
    )

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    rospy
    senseglove_shared_resources
    std_msgs
    actionlib
    control_msgs
    trajectory_msgs
    dynamic_reconfigure
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
