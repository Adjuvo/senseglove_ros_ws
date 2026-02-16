from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    type = LaunchConfiguration('type').perform(context)
    side = LaunchConfiguration('side').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)

    urdf_dir = os.path.join(get_package_share_directory('senseglove_description'), 'urdf', type)
    sides = ['left', 'right'] if side == 'both' else [side]

    nodes = []

    for s in sides:
        urdf_file = os.path.join(urdf_dir, f'{type}_{s}.urdf.xacro')

        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{s}',
            namespace=s,
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', urdf_file,
                    ' prefix:=', prefix
                ])
            }]
        ))

        nodes.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name=f'joint_state_publisher_gui_{s}',
            namespace=s,
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', urdf_file,
                    ' prefix:=', prefix
                ])
            }]
        ))

        # Publish robot_description once as a topic
        nodes.append(ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub',
                '--qos-durability=transient_local',
                '--once',
                f'/robot_description_{s}', 'std_msgs/msg/String',
                ["data: '", Command(['xacro ', urdf_file, ' prefix:=', prefix]), "'"]
            ],
            output='log'
        ))

    # RViz
    rviz_dir = os.path.join(get_package_share_directory('senseglove_description'), 'rviz')
    rviz_config = os.path.join(rviz_dir, f'urdf_{side}.rviz')

    if os.path.exists(rviz_config):
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ))
    else:
        print(f"[WARN] No RViz config found for side '{side}' at {rviz_config}")

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'type',
            default_value='nova2',
            description='Which SenseGlove type to use: dk1, nova, nova2'
        ),
        DeclareLaunchArgument(
            'side',
            default_value='right',
            description='Which side to use: left, right, or both'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for joint names'
        ),
        OpaqueFunction(function=launch_setup)
    ])
