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

    controller_yaml_file = PathJoinSubstitution([
        FindPackageShare('senseglove_hardware_interface'),
        'config',
        robot,
        'controllers.yaml'
    ])

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[robot_description, controller_yaml_file],
        output='screen'
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60']
    )

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['trajectory_controller', '--controller-manager-timeout', '60']
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[robot_description, {'publish_frequency': 60}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot', description='The robot model to use'),
        DeclareLaunchArgument('gloveIndex', description='Index of the glove'),
        DeclareLaunchArgument('isRight', description='Is right hand glove? (true/false)'),
        ros2_control_node,
        controller_spawner,
        trajectory_controller_spawner,
        robot_state_publisher
    ])
