from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory
import yaml
import os

_RESET  = '\033[0m'
_YELLOW = '\033[1;33m'

def generate_launch_description():
    run_rviz = LaunchConfiguration('run_rviz')
    run_sensecom = LaunchConfiguration('run_sensecom')
    run_finger_distance = LaunchConfiguration('run_finger_distance')

    env_colorize = SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1')
    env_format = SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='[{severity}] [{name}]: {message}')

    # Locate senseglove_com package
    sensecom_share = get_package_share_directory('senseglove_com')
    sensecom_bin = os.path.join(
        sensecom_share, 'Linux', 'SenseCom_Linux_Latest', 'SenseCom.x86_64'
    )

    # Start SenseCom
    sensecom_process = ExecuteProcess(
        cmd=[sensecom_bin],
        output='log',
        condition=IfCondition(run_sensecom)
    )

    # Locate senseglove_bringup package
    launch_share = get_package_share_directory('senseglove_bringup')    
    gloves_file = os.path.join(launch_share, 'config', 'gloves.yaml')
    finger_distance_launch = os.path.join(launch_share, 'launch', 'finger_distance.launch.py')

    # Locate senseglove_hardware_interface package
    hw_share = get_package_share_directory('senseglove_hardware_interface')    
    hardware_launch = os.path.join(hw_share, 'launch', 'hardware.launch.py')

    # Load gloves configuration
    with open(gloves_file, 'r') as f:
        config = yaml.safe_load(f)
    gloves = config.get('gloves', [])

    def launch_hardware_nodes(context, *args, **kwargs):
        input(f"{_YELLOW}Start SenseCom. Confirm all gloves connected, then press ENTER...{_RESET}")

        hardware_nodes = []
        for glove in gloves:
            robot = glove.get('type', 'nova2') + '_' + glove.get('side', 'left')
            is_right = 'true' if glove.get('side') == 'right' else 'false'
            glove_serial = str(glove.get('serial', ''))

            launch_args = {
                'robot': robot,
                'isRight': is_right,
                'gloveSerial': glove_serial
            }

            hardware_nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(hardware_launch),
                    launch_arguments=launch_args.items()
                )
            )

        return hardware_nodes

    # Hardware Nodes Event handlers
    launch_hardware_nodes_with_sensecom = RegisterEventHandler(
        OnProcessStart(
            target_action=sensecom_process,
            on_start=[OpaqueFunction(function=launch_hardware_nodes)]
        )
    )

    launch_hardware_nodes_without_sensecom = OpaqueFunction(
        function=launch_hardware_nodes,
        condition=UnlessCondition(run_sensecom)
    )

    # Finger Tip Distance Nodes
    launch_finger_distance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(finger_distance_launch),
        condition=IfCondition(run_finger_distance)
    )

    # Locate senseglove_description package
    description_share = get_package_share_directory('senseglove_description')

    if len(gloves) == 2:
        rviz_file = os.path.join(description_share, 'rviz', 'urdf_both.rviz')
    elif len(gloves) == 1:
        side = gloves[0].get('side', 'left')
        rviz_file = os.path.join(description_share, 'rviz', f'urdf_{side}.rviz')
    else:
        rviz_file = os.path.join(description_share, 'rviz', 'urdf_left.rviz')

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
        condition=IfCondition(run_rviz)
    )

    return LaunchDescription([
        env_colorize,
        env_format,
        DeclareLaunchArgument('run_rviz', default_value='false', choices=['true', 'false'], description='Launch RViz'),
        DeclareLaunchArgument('run_sensecom', default_value='false', choices=['true', 'false'], description='Start SenseCom executable'),
        DeclareLaunchArgument('run_finger_distance', default_value='false', choices=['true', 'false'], description='Start Finger-Tip Distance Nodes'),
        sensecom_process,
        launch_hardware_nodes_with_sensecom,
        launch_hardware_nodes_without_sensecom,
        launch_finger_distance,
        rviz_node
    ])
