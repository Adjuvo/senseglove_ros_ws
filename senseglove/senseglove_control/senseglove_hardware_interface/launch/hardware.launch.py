from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from launch.event_handlers.on_process_start import OnProcessStart

from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, LogInfo


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    is_right = LaunchConfiguration('isRight')
    glove_serial = LaunchConfiguration('gloveSerial')
    robot_type = PythonExpression(["'", robot, "'.split('_')[0]"])
    handedness = PythonExpression(['"rh" if "', is_right, '" == "true" else "lh"'])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('senseglove_description'),
        'urdf',
        robot_type,
        [robot, TextSubstitution(text='.urdf.xacro')]
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        " ",
        xacro_file,
        " selected_robot:=", robot,
        " is_right:=", is_right,
        " glove_serial:=", glove_serial,
        " publish_rate:=", "60",
    ])

    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('senseglove_hardware_interface'),
        'config',
        robot,
        'controllers.yaml'
    ])

    namespace = PathJoinSubstitution([
        '/senseglove/',
        PythonExpression(["'glove' + '", glove_serial, "' + '/' + ('rh' if '", is_right, "' == 'true' else 'lh')"]),
    ])

    def log_glove(context, *args, **kwargs):
        robot = LaunchConfiguration('robot').perform(context)
        is_right = LaunchConfiguration('isRight').perform(context)
        glove_serial = LaunchConfiguration('gloveSerial').perform(context)

        return [
            LogInfo(
                msg=f"[SenseGlove] Launching: robot={robot} serial={glove_serial} is_right={is_right}"
            )
        ]

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        arguments=[
        '--ros-args',
        '--log-level', 'resource_manager:=WARN',
        ],
        output='screen',
        namespace=namespace
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        namespace=namespace
    )

    senseglove_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'senseglove_state_broadcaster',
            '--param-file', robot_controllers
        ],
        output='screen',
        namespace=namespace
    )

    haptics_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'haptics_controller',
            '--param-file', robot_controllers
        ],
        output='screen',
        namespace=namespace
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
        namespace=namespace
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot', description='The robot model to use'),
        DeclareLaunchArgument('isRight', description='Whether this is a right hand glove (true/false)'),
        DeclareLaunchArgument('gloveSerial', description='Serial number of the glove'),
        OpaqueFunction(function=log_glove),
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        senseglove_state_broadcaster_spawner,
        haptics_controller_spawner
    ])