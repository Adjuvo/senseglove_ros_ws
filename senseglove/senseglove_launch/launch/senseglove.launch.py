from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml
import os

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # Locate this package's share dir
    package_share = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share', 'senseglove_launch'
    )
    config_dir = os.path.join(package_share, 'config')

    # Start SenseCom
    sensecom_process = ExecuteProcess(
        cmd=[
            os.path.join(
                os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                'share', 'senseglove_com', 'SenseCom', 'Linux', 'SenseCom_Linux_Latest', 'SenseCom.x86_64'
            )
        ],
        output='screen'
    )

    # Wait for user confirmation
    wait_for_user = ExecuteProcess(
        cmd=['python3', '-c', "input('Please confirm all gloves are connected in SenseCom and press ENTER to continue...')"],
        output='screen'
    )

    # Parse gloves.yaml
    gloves_file = os.path.join(config_dir, 'gloves.yaml')
    with open(gloves_file, 'r') as f:
        config = yaml.safe_load(f)
    gloves = config.get('gloves', [])

    hardware_nodes = []
    for glove in gloves:
        robot = glove.get('type', 'nova2') + '_' + glove.get('side', 'left')
        index = str(glove.get('index', 0))
        is_right = 'true' if glove.get('side') == 'right' else 'false'

        hardware_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.dirname(__file__),
                        'hardware.launch.py'
                    )
                ),
                launch_arguments={
                    'robot': robot,
                    'gloveIndex': index,
                    'isRight': is_right
                }.items()
            )
        )

    # Calibration nodes
    calib_left_file = os.path.join(config_dir, 'calibration_left.yaml')
    calib_right_file = os.path.join(config_dir, 'calibration_right.yaml')

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

    # RViz
    rviz_node = Node(
        condition=use_rviz,
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0],
            'share', 'senseglove_description', 'config', 'urdf_both.rviz'
        )],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),

        sensecom_process,
        wait_for_user,
        LogInfo(msg='Launching gloves...'),

        *hardware_nodes,
        calibration_left,
        calibration_right,
        rviz_node
    ])
