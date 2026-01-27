import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Accel

import random

class PublishRandomTargetPositions(Node):
    def __init__(self):
        super().__init__("publish_random_target_positions")

        self.timer = self.create_timer(3, self.publish_random_target)

        self.target_publisher = self.create_publisher(
            PoseStamped, "target_pos",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
    
    def publish_random_target(self):

        current_time_stamp_msg = self.get_clock().now().to_msg()
        frame_id = self.get_namespace().lstrip('/')

        msg = PoseStamped()
        msg.header.stamp = current_time_stamp_msg
        msg.header.frame_id = frame_id

        msg.pose.position.x = float(random.randrange(-2, 2+1))
        msg.pose.position.y = float(random.randrange(-2, 2+1))
        msg.pose.position.z = float(0)

        self.get_logger().warn(f"publishing random target position: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})")

        self.target_publisher.publish(msg)





def main(args=None):
    rclpy.init(args=args)
    node = PublishRandomTargetPositions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
