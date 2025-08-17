from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory
import yaml
import os

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # Locate senseglove_com package
    sensecom_share = get_package_share_directory('senseglove_com')
    sensecom_bin = os.path.join(
        sensecom_share, 'Linux', 'SenseCom_Linux_Latest', 'SenseCom.x86_64'
    )

    # Start SenseCom
    sensecom_process = ExecuteProcess(
        cmd=[sensecom_bin],
        output='log'
    )

    # Locate senseglove_launch package
    launch_share = get_package_share_directory('senseglove_launch')    
    gloves_file = os.path.join(launch_share, 'config', 'gloves.yaml')

    # Locate senseglove_hardware_interface package
    hw_share = get_package_share_directory('senseglove_hardware_interface')    
    hardware_launch = os.path.join(hw_share, 'launch','hardware.launch.py')

    # Load gloves configuration
    with open(gloves_file, 'r') as f:
        config = yaml.safe_load(f)
    gloves = config.get('gloves', [])

    def launch_hardware_nodes(context, *args, **kwargs):
        input(" Starting SenseCom. Please confirm all gloves are connected in SenseCom and press ENTER to continue...")
        
        hardware_nodes = []
        for glove in gloves:
            robot = glove.get('type', 'nova2') + '_' + glove.get('side', 'left')
            index = str(glove.get('index', 0))
            is_right = 'true' if glove.get('side') == 'right' else 'false'

            hardware_nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(hardware_launch),
                    launch_arguments={
                        'robot': robot,
                        'gloveIndex': index,
                        'isRight': is_right
                    }.items()
                )
            )
        
        return hardware_nodes

    # Hardware Nodes Event handler
    launch_hardware_nodes_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=sensecom_process,
            on_start=[OpaqueFunction(function=launch_hardware_nodes)]
        )
    )

    # Calibration nodes
    calib_left_file = os.path.join(launch_share, 'config', 'calibration_left.yaml')
    calib_right_file = os.path.join(launch_share, 'config', 'calibration_right.yaml')

    # TODO: This needs a dynamic arg call
    calibration_left = Node(
        package='senseglove_interaction',
        executable='senseglove_finger_distance_node',
        name='senseglove_finger_distance_left',
        output='screen',
        arguments=['0', 'normalized'],
        parameters=[calib_left_file]
    )

    calibration_right = Node(
        package='senseglove_interaction',
        executable='senseglove_finger_distance_node',
        name='senseglove_finger_distance_right',
        output='screen',
        arguments=['1', 'normalized'],
        parameters=[calib_right_file]
    )

    # Locate senseglove_description package
    description_share = get_package_share_directory('senseglove_description')    
    rviz_right_file = os.path.join(description_share, 'rviz', 'urdf_right.rviz')

    # RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_right_file],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        sensecom_process,
        launch_hardware_nodes_handler,
        # calibration_left,
        # calibration_right,
        rviz_node
    ])
