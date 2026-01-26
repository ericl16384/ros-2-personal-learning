import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'layered_control_systems'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'srv'), glob('urdf/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ericl',
    maintainer_email='ericl16384@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_hardware_simulator = layered_control_systems.arm_hardware_simulator:main',
            'mocap_simulator = layered_control_systems.mocap_simulator:main',
            'arm_controller = layered_control_systems.arm_controller:main',

            'publish_random_arm_target_positions = layered_control_systems.publish_random_arm_target_positions:main',
        ],
    },
)
