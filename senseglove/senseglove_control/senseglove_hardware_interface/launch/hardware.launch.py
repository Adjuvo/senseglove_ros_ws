from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression

def generate_launch_description():

    robot = LaunchConfiguration('robot')
    glove_index = LaunchConfiguration('gloveIndex')
    is_right = LaunchConfiguration('isRight')

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
        " ",
        "selected_robot:=", robot,
        " ",
        "glove_index:=", glove_index,
        " ",
        "is_right:=", is_right,
        " ",
        "publish_rate:=", "100", 
        ])


    robot_description = {'robot_description': robot_description_content}

    namespace = PathJoinSubstitution([
        '/senseglove/',
        PythonExpression(["'glove' + str(", glove_index, ")"]),
        handedness
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('senseglove_hardware_interface'),
        'config',
        robot,
        'controllers.yaml'
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen',
        namespace=namespace
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='screen',
        namespace=namespace
    )

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['trajectory_controller', '--controller-manager-timeout', '60'],
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
        DeclareLaunchArgument('gloveIndex', description='Index of the glove'),
        DeclareLaunchArgument('isRight', description='Is right hand glove? (true/false)'),
        control_node,
        controller_spawner,
        trajectory_controller_spawner,
        robot_state_publisher
    ])
