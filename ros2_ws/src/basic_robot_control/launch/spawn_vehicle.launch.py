import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # -----------------------------------------------------------------------
    # PATHS - CHANGE 'vehicle_blue_pkg' TO YOUR ACTUAL PACKAGE NAME
    # -----------------------------------------------------------------------
    pkg_name = 'basic_robot_control'
    
    # If you haven't built a package yet and just want to point to a file,
    # uncomment the line below and put the full path to your URDF:
    # urdf_file_path = "/home/user/ros2_ws/src/vehicle_blue/urdf/vehicle_blue.urdf"
    
    # Otherwise, we assume standard ROS 2 package structure:
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'vehicle_blue.urdf')

    # Read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # -----------------------------------------------------------------------
    # LAUNCH CONFIGURATIONS
    # -----------------------------------------------------------------------
    
    # Start Gazebo Sim with an empty world
    # We use the standard ros_gz_sim launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )]
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # -----------------------------------------------------------------------
    # NODES
    # -----------------------------------------------------------------------

    # 1. Robot State Publisher
    # Publishes the TF tree so Rviz and Gazebo know where links are
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. Spawn Entity
    # Sends the URDF to Gazebo to create the visual/physical model
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', # Read from the topic published by RSP above
            '-name', 'vehicle_blue',
            '-z', '0.5' # Spawn slightly above ground to prevent clipping
        ],
        output='screen'
    )

    # 3. ROS-Gazebo Bridge
    # This is crucial for testing friction!
    # It bridges the /cmd_vel topic so you can drive the robot from ROS.
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge cmd_vel (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Bridge odometry (Gazebo -> ROS)
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # Bridge joint states (Gazebo -> ROS)
            '/world/empty/model/vehicle_blue/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        node_spawn_entity,
        node_ros_gz_bridge,
    ])