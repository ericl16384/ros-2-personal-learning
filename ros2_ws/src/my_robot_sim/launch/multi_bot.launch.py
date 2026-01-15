import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'my_robot_sim'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 1. Configuration Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )

    # 2. Process Xacro (convert .xacro to .xml string for the simulation)
    # We use Command() so this happens at runtime
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 3. Start Gazebo Environment
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    # 4. Helper Function to Spawn a Robot
    # This wraps everything in a Namespace so nodes don't clash
    def spawn_robot(name, x, y):
        return GroupAction([
            PushRosNamespace(name),
            
            # A. Publish the Robot State (TF Tree)
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}]
            ),

            # B. Spawn the Entity in Gazebo
            # We must pass the namespace to Gazebo so the plugins (ros2_control) connect correctly
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                output='screen',
                arguments=[
                    '-topic', 'robot_description', 
                    '-entity', name, 
                    '-x', str(x), '-y', str(y),
                    '-robot_namespace', name
                ]
            ),

            # C. Start the Joint State Broadcaster (Reads encoders)
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
            ),

            # D. Start the Mecanum Controller (Reads cmd_vel)
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["mecanum_cont", "--controller-manager", "controller_manager"],
            ),
        ])

    # 5. Return the Final Launch Description
    return LaunchDescription([
        gazebo,
        # Spawn Robot 1 at (0, 0)
        spawn_robot('robot1', 0.0, 0.0),
        # Spawn Robot 2 at (2, 0)
        spawn_robot('robot2', 2.0, 0.0),
    ])