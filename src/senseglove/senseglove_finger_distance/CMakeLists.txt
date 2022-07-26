cmake_minimum_required(VERSION 3.0.2)
project(senseglove_finger_distance)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    genmsg
    rospy
    senseglove_shared_resources
    std_msgs
    )

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    senseglove_shared_resources
    std_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node scripts/senseglove_haptics_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
