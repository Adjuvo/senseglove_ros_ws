# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=[
        'senseglove_interaction',
        'senseglove_interaction.finger_distance',
        'senseglove_interaction.haptics',
        'senseglove_interaction.common'
    ],
    package_dir={'': 'src'},
    scripts=['scripts/senseglove_finger_distance_node',
             'scripts/senseglove_haptics_node',
             'src/senseglove_interaction/finger_distance/finger_distance_calibration.py',
             'src/senseglove_interaction/haptics/haptics_node_simple.py',
             'src/senseglove_interaction/haptics/haptics_node_dynamic.py',
             'src/senseglove_interaction/common/imu_tf_broadcaster.py']
             )
setup(**setup_args)
