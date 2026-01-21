import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Accel
from std_srvs.srv import SetBool # <--- Service Type

import numpy as np

class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller")

        self.target_position = np.array((1, 2, 0))

        self.acceleration_command = Accel()
        
        self.arm_head_pose_subscription = self.create_subscription(
            PoseStamped, "arm_head_pose_stamped", self.head_pose_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.arm_head_command_publisher = self.create_publisher(
            Accel, "arm_head_command_accel",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
    
    def head_pose_callback(self, msg:PoseStamped):
        position = np.array((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

        displacement = self.target_position - position

        acceleration = displacement * 0.1

        self.acceleration_command.linear.x = acceleration[0]
        self.acceleration_command.linear.y = acceleration[1]
        self.acceleration_command.linear.z = acceleration[2]

        self.publish_command()
        
    def publish_command(self):
        msg = Accel()
        
        msg.linear.x = self.acceleration_command.linear.x
        msg.linear.y = self.acceleration_command.linear.y
        msg.linear.z = self.acceleration_command.linear.z

        self.arm_head_command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


    