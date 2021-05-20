# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=['senseglove_finger_distance'],
    package_dir={'': 'src'},
    scripts=['scripts/senseglove_finger_distance_node', 'scripts/senseglove_haptics_node',
             'src/senseglove_finger_distance/finger_distance_calibration.py'],
)

setup(**setup_args)
