# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=['senseglove_haptics'],
    package_dir={'': 'src'},
    scripts=['scripts/senseglove_haptics_node'],
)

setup(**setup_args)
