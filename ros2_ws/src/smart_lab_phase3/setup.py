from setuptools import find_packages, setup

package_name = 'smart_lab_phase3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'drone_hardware = smart_lab_phase3.drone_hardware:main',
            'flight_computer = smart_lab_phase3.flight_computer:main',
            'mocap_simulator = smart_lab_phase3.mocap_simulator:main',
        ],
    },
)
