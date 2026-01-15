import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Include all URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # Include all Config/YAML files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ericl',
    maintainer_email='ericl16384@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # This is where we register your control scripts later
            'formation_control = my_robot_sim.formation_control:main',
        ],
    },
)