from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smart_lab_phase3',
            executable='mocap_simulator',
            name='vicon',
            output='screen'
        ),

        # --- ALPHA DRONE ---
        # Hardware Node (Sensor Data)
        Node(
            package="smart_lab_phase3",  # Must match your new package name
            executable="drone_hardware", # Must match entry_point in setup.py
            namespace="alpha",           # This prefixes topics: /alpha/imu
            name="hardware_node",        # Custom name for logs
            output="screen"
        ),
        # Flight Computer (Logic)
        Node(
            package="smart_lab_phase3",
            executable="flight_computer",
            namespace="alpha",           # Matches the hardware above
            name="computer_node",
            output="screen"
        ),

        # --- BRAVO DRONE ---
        # Hardware Node
        Node(
            package="smart_lab_phase3",
            executable="drone_hardware",
            namespace="bravo",           # Prefixes topics: /bravo/imu
            name="hardware_node",
            output="screen"
        ),
        # Flight Computer
        Node(
            package="smart_lab_phase3",
            executable="flight_computer",
            namespace="bravo",
            name="computer_node",
            output="screen"
        ),
    ])