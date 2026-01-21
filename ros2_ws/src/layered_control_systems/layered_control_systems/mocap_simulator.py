import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster

class MocapSimulator(Node):
    def __init__(self):
        super().__init__('mocap_simulator')
        
        self.arm_head_command_subscription = self.create_subscription(
            PoseStamped, "arm_head_pose_stamped", self.broadcast_arm_head_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # The Broadcaster is the tool that sends coordinates to the ROS network
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Update positions at 20Hz (similar to a slow mocap system)
        # self.timer = self.create_timer(0.05, self.broadcast_positions)
        self.get_logger().info("Simulation Started: Broadcasting TF frames.")
    
    def broadcast_arm_head_callback(self, msg:PoseStamped):
        transform = TransformStamped()

        # transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.stamp = msg.header.stamp

        transform.header.frame_id = 'world'
        transform.child_frame_id = 'arm_head'

        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        
        # self.get_logger().info(f"publishing transform: {transform.transform.translation}")

        self.tf_broadcaster.sendTransform([transform])

    # def broadcast_positions(self):
    #     # Get the current ROS time (essential for TF to work)
    #     now = self.get_clock().now().to_msg()
        
    #     # --- 1. DEFINE ALPHA'S LOCATION ---
    #     t_alpha = TransformStamped()
        
    #     # The Header
    #     t_alpha.header.stamp = now
    #     t_alpha.header.frame_id = 'world'  # Parent (The Room)
    #     t_alpha.child_frame_id = 'alpha'   # Child (The Robot)
        
    #     # The Translation (x, y, z in meters)
    #     t_alpha.transform.translation.x = 2.0
    #     t_alpha.transform.translation.y = 0.0
    #     t_alpha.transform.translation.z = 1.0  # 1 meter altitude
        
    #     # The Rotation (Quaternion). w=1.0 means "No Rotation" (Level)
    #     t_alpha.transform.rotation.w = 1.0 

    #     # --- 2. DEFINE BRAVO'S LOCATION ---
    #     t_bravo = TransformStamped()
        
    #     t_bravo.header.stamp = now
    #     t_bravo.header.frame_id = 'world'
    #     t_bravo.child_frame_id = 'bravo'
        
    #     # Position (Opposite side of the room)
    #     t_bravo.transform.translation.x = -2.0
    #     t_bravo.transform.translation.y = 0.0
    #     t_bravo.transform.translation.z = 1.0
    #     t_bravo.transform.rotation.w = 1.0

    #     # --- 3. SEND BOTH ---
    #     # We can send a list of transforms at once
    #     self.tf_broadcaster.sendTransform([t_alpha, t_bravo])

def main(args=None):
    rclpy.init(args=args)
    node = MocapSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
