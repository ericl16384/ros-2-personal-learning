from import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool # <--- Service Type

class ArmController(Node):
    def __init__(self):
        super().__init__("arm-controller")

    