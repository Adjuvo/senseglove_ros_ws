cmake_minimum_required(VERSION 3.0.2)
project(senseglove_launch)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(PROGRAMS scripts/startGlove.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})