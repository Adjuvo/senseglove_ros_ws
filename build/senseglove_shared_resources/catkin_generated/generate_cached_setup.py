# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/rogier/march_ws/install/march_tutorials;/home/rogier/march_ws/install/march_temperature_sensor_controller;/home/rogier/march_ws/install/march_launch;/home/rogier/march_ws/install/march_state_machine;/home/rogier/march_ws/install/march_simulation;/home/rogier/march_ws/install/march_rqt_gait_generator;/home/rogier/march_ws/install/march_gait_selection;/home/rogier/march_ws/install/march_shared_classes;/home/rogier/march_ws/install/march_safety;/home/rogier/march_ws/install/march_monitor;/home/rogier/march_ws/install/march_rqt_robot_monitor;/home/rogier/march_ws/install/march_rqt_note_taker;/home/rogier/march_ws/install/march_rqt_input_device;/home/rogier/march_ws/install/march_pdb_state_controller;/home/rogier/march_ws/install/march_joint_inertia_controller;/home/rogier/march_ws/install/march_hardware_interface;/home/rogier/march_ws/install/march_gazebo_plugins;/home/rogier/march_ws/install/march_gait_scheduler;/home/rogier/march_ws/install/march_gain_scheduling;/home/rogier/march_ws/install/march_data_collector;/home/rogier/march_ws/install/march_shared_resources;/home/rogier/march_ws/install/march_rqt_software_check;/home/rogier/march_ws/install/march_rqt_gait_selection;/home/rogier/march_ws/install/march_moveit;/home/rogier/march_ws/install/march_imu_manager;/home/rogier/march_ws/install/march_identification;/home/rogier/march_ws/install/march_hardware_builder;/home/rogier/march_ws/install/march_hardware;/home/rogier/march_ws/install/march_gait_files;/home/rogier/march_ws/install/march_fake_sensor_data;/home/rogier/march_ws/install/march_ems_projects;/home/rogier/march_ws/install/march_description;/opt/ros/melodic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/rogier/SenseGlove_ROS/ros_workspace/build/senseglove_shared_resources/devel/env.sh')

output_filename = '/home/rogier/SenseGlove_ROS/ros_workspace/build/senseglove_shared_resources/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
