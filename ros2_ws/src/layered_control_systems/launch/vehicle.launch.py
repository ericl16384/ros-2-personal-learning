import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# import os
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # pkg_path = get_package_share_directory('layered_control_systems')
    # rviz_config_path = os.path.join(pkg_path, 'rviz', 'arm.rviz')

    return LaunchDescription([
        # # # # --- RVIZ (Pre-configured) ---
        # # # Node(
        # # #     package='rviz2',
        # # #     executable='rviz2',
        # # #     name='rviz2',
        # # #     output='screen',
        # # #     arguments=['-d', rviz_config_path] # -d means "load description file"
        # # # ),
        
        Node(
            package='layered_control_systems',
            executable='drivetrain_hardware_simulator',
            namespace='vehicle_1',
            # name='arm_hardware_simulator',
            output='screen'
        ),
        Node(
            package='layered_control_systems',
            executable='arm_hardware_simulator',
            namespace='vehicle_1',
            # name='arm_hardware_simulator',
            output='screen'
        ),
        
        Node(
            package='layered_control_systems',
            executable='mocap_simulator',
            namespace='vehicle_1',
            # name='mocap_simulator',
            output='screen'
        ),

        Node(
            package='layered_control_systems',
            executable='vehicle_manager',
            namespace='vehicle_1',
            # name='arm_controller',
            output='screen'
        ),
        Node(
            package='layered_control_systems',
            executable='arm_controller',
            namespace='vehicle_1',
            # name='arm_controller',
            output='screen'
        ),
        Node(
            package='layered_control_systems',
            executable='drivetrain_controller',
            namespace='vehicle_1',
            # name='arm_controller',
            output='screen'
        ),
        
        # Node(
        #     package='layered_control_systems',
        #     executable='publish_random_target_positions',
        #     namespace='vehicle_1',
        #     # name='arm_controller',
        #     output='screen'
        # ),
    ])