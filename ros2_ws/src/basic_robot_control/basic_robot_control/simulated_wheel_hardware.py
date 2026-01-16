from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

import numpy as np

class SimulatedWheelHardware(Node):
    def __init__(self):
        super().__init__('simulated_wheel_hardware')
        
        self.timer = self.create_timer(0.01, self.publish_sensor_data)
        
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_publisher(JointState, "wheel_position", qos_profile)

        self.current_position = np.zeros(3)
    
    def publish_sensor_data(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.position = self.get_encoder_position()
        msg.velocity = self.get_encoder_velocity()
    
    def get_encoder_position(self):
        return self.
    
    def get_encoder_velocity(self):
        return 0.0
    
    def get_motor_torque(self):
        return 0.0
