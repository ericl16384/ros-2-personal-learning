import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Vector3Stamped

import random

class PublishRandomArmPositionTargets(Node):
    def __init__(self):
        super().__init__("publish_random_arm_target_positions")

        self.timer = self.create_timer(2, self.publish_random_target)

        self.arm_head_command_publisher = self.create_publisher(
            Vector3Stamped, "arm_head_target_pos",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
    
    def publish_random_target(self):

        current_time_stamp_msg = self.get_clock().now().to_msg()
        frame_id = self.get_namespace().lstrip('/')

        msg = Vector3Stamped()
        msg.header.stamp = current_time_stamp_msg
        msg.header.frame_id = frame_id

        msg.vector.x = float(random.randrange(-2, 2+1))
        msg.vector.y = float(random.randrange(-2, 2+1))
        msg.vector.z = float(0)

        self.get_logger().warn(f"publishing random target position: ({msg.vector.x}, {msg.vector.y}, {msg.vector.z})")

        self.arm_head_command_publisher.publish(msg)





def main(args=None):
    rclpy.init(args=args)
    node = PublishRandomArmPositionTargets()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
