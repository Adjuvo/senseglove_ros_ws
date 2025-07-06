from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression


import os
from ament_index_python.packages import get_package_share_directory
from xacro import process_file

def generate_launch_description():
    # Launch args
    robot = LaunchConfiguration('robot')
    glove_index = LaunchConfiguration('gloveIndex')
    is_right = LaunchConfiguration('isRight')

    robot_type = PythonExpression(["'", robot, "'.split('_')[0]"])
    handedness = PythonExpression(['"rh" if "', is_right, '" == "true" else "lh"'])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('senseglove_description'),
        'urdf', robot_type, TextSubstitution(text='/'), robot, '.xacro'
    ])

    robot_description = {
        'robot_description': Command(['xacro', xacro_file])
    }

    namespace = PathJoinSubstitution([
        '/senseglove/glove',
        PythonExpression(['str(', glove_index, ')']),
        handedness
    ])

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[robot_description, {'publish_frequency': 60}]
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster', 'trajectory_controller']
    )

    hardware_interface = Node(
        package='senseglove_hardware_interface',
        executable='senseglove_hardware_interface_node',
        namespace=namespace,
        arguments=[robot, glove_index, is_right]
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot', description='The robot model to use'),
        DeclareLaunchArgument('gloveIndex', description='Index of the glove'),
        DeclareLaunchArgument('isRight', description='Is right hand glove? (true/false)'),
        robot_state_publisher,
        controller_spawner,
        hardware_interface
    ])
