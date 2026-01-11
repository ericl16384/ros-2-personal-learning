import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
import random
import time

class DroneHardware(Node):
    def __init__(self):
        super().__init__("drone_hardware")
        
        # QUALITY OF SERVICE: 
        # "Best Effort" is standard for high-frequency sensor data on drones.
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.publisher_ = self.create_publisher(Imu, "/drone/imu", qos_profile)
        self.timer = self.create_timer(0.1, self.publish_sensor_data) # 10Hz
        self.get_logger().info("Drone Hardware Node Started (Simulating IMU)")

    def publish_sensor_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Simulate Roll/Pitch (orientation) data with noise
        # In a real driver, this is where you'd parse serial bytes
        # Simulating a dangerous tilt (over 0.5 rad) occasionally
        msg.orientation.x = random.uniform(-0.1, 0.1) + (0.6 if random.random() > 0.95 else 0.0)
        msg.orientation.y = random.uniform(-0.1, 0.1)
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneHardware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()