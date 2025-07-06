from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterFile
import os

def generate_launch_description():
    # Launch args
    robot = LaunchConfiguration('robot')
    glove_index = LaunchConfiguration('gloveIndex')
    is_right = LaunchConfiguration('isRight')

    # Build namespace
    ns = [ '/senseglove/glove', 
           (glove_index.perform({}) if glove_index else '0'),
           '/rh' if is_right.perform({}) == 'true' else '/lh']

    namespace = ''.join(ns)

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': open(
                os.path.join(
                    os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                    'share', 'senseglove_description', 'urdf',
                    f'{robot.perform({})}.urdf'
                )
            ).read(),
            'publish_frequency': 60
        }]
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
