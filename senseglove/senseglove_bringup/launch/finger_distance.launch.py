from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    # Locate senseglove_bringup package
    launch_share = get_package_share_directory('senseglove_bringup')    
    gloves_file = os.path.join(launch_share, 'config', 'gloves.yaml')
    calib_path = os.path.join(launch_share, 'config', "calibration.yaml")

    # Load gloves configuration
    with open(gloves_file, 'r') as f:
        config = yaml.safe_load(f)
    gloves = config.get('gloves', [])

    # Finger Distance Nodes
    def launch_finger_distance_nodes(context, *args, **kwargs):
        finger_distance_nodes = []
        for glove in gloves:
            robot = glove.get('type', 'nova2')
            glove_index = int(glove.get('index', 0))
            side = glove.get('side', 'right')
            is_right = (side == 'right')
            use_finger_distance = glove.get('finger_distance', True)

            if not use_finger_distance:
                continue

            finger_distance_nodes.append(
                Node(
                    package='senseglove_interaction',
                    executable='finger_tip_distance_node',
                    name="finger_tip_distance_node",
                    parameters=[calib_path,
                                {'calib_mode': 'nothing'},
                    ],
                    namespace=f"/senseglove/glove{glove_index}/{'rh' if is_right else 'lh'}",
                    output='screen',
                )
            )
        
        return finger_distance_nodes

    return LaunchDescription([
        OpaqueFunction(function=launch_finger_distance_nodes),
    ])
