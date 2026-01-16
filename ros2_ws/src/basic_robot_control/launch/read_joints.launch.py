# import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # pkg_path = get_package_share_directory('smart_lab_phase3')
    # rviz_config_path = os.path.join(pkg_path, 'rviz', 'fleet.rviz')

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
            output='screen'
        ),
    ])