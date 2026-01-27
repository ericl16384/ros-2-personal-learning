import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Accel

import numpy as np




class VehicleManager(Node):
    def __init__(self):
        super().__init__("vehicle_manager")

        self.target_subscription = self.create_subscription(
            PoseStamped, "target_pos", self.target_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.drivetrain_target_publisher = self.create_publisher(
            PoseStamped, "drivetrain_target_pos",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        # self.arm_target_publisher = self.create_publisher(
        #     Vector3Stamped, "arm_target_pos",
        #     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # )
    
    def target_callback(self, msg):
        self.drivetrain_target_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
