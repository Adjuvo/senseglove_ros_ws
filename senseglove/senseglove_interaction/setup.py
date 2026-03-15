from setuptools import setup

package_name = 'senseglove_interaction'

setup(
    name=package_name,
    version='3.0.0',
    description='SenseGlove interaction nodes',
    url='https://senseglove.com',
    author='Akshay Radhamohan Menon',
    author_email='akshay@senseglove.com',
    license='MIT',
    packages=[
        package_name,
        f'{package_name}.finger_distance',
        f'{package_name}.haptics',
        f'{package_name}.common',
    ],
    package_dir={'': '.'},
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'action_msgs',
        'senseglove_msgs',
        'tf_transformations',
    ],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            # finger distance
            'finger_tip_distance_node = senseglove_interaction.finger_distance.finger_distance_node:main',
            'finger_tip_distance_calibration = senseglove_interaction.finger_distance.finger_distance_calibration:main',
            'finger_tip_distance_planar_node = senseglove_interaction.finger_distance.finger_tip_distance_planar_node:main',
            'calibration_manager = senseglove_interaction.finger_distance.calibration_manager:main',
            # haptics nodes
            'haptics_node = senseglove_interaction.haptics.haptics_node:main',
            'nova2_vibration_node = senseglove_interaction.haptics.nova2_vibration_node:main',
            # imu tf broadcaster
            'imu_tf_broadcaster = senseglove_interaction.common.imu_tf_broadcaster:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
)
