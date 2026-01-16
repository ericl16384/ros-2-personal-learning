import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped

class WheelPoseListener(Node):
    def __init__(self):
        super().__init__('wheel_pose_listener')

        # 1. Declare frames
        # 'source_frame': The frame you want to locate (the wheel)
        # 'target_frame': The reference frame (the world or robot base)
        self.target_frame = 'map'          # Or 'odom', 'world'
        self.source_frame = 'wheel_left_link' # Replace with your actual link name

        # 2. Create a buffer and listener to capture TF data
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Create a timer to query the pose repeatedly (e.g., 10Hz)
        self.timer = self.create_timer(0.1, self.get_pose)
        
        self.get_logger().info(f"Listening for transform: {self.target_frame} -> {self.source_frame}")

    def get_pose(self):
        try:
            # Look up the latest available transform
            # to_frame_rel, from_frame_rel, time, timeout
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                self.get_clock().now().to_msg())

            # Extract Position
            pos = t.transform.translation
            # Extract Orientation (Quaternion)
            rot = t.transform.rotation

            self.get_logger().info(
                f'\nWheel Pose in {self.target_frame}:'
                f'\n  Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}'
                f'\n  Rotation: x={rot.x:.2f}, y={rot.y:.2f}, z={rot.z:.2f}, w={rot.w:.2f}'
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WheelPoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()