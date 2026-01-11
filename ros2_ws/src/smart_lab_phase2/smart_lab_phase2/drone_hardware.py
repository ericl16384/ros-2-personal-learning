import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool  # <--- Service Type
import random

class NetworkedDroneHardware(Node):
    def __init__(self):
        super().__init__("networked_drone_hardware")
        
        # Publisher Setup (Best Effort for sensors)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(Imu, "drone/imu", qos_profile)
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        
        # State Variable
        self.armed = True

        # SERVICE SERVER: Listens for commands on "/drone/set_armed"
        self.srv = self.create_service(SetBool, "drone/set_armed", self.set_armed_callback)
        
        self.get_logger().info("Networked Drone Hardware Live. Service /drone/set_armed ready.")

    def set_armed_callback(self, request, response):
        # request.data is True (Arm) or False (Disarm)
        if request.data:
            self.armed = True
            response.message = "MOTORS ARMED"
            self.get_logger().info("Received ARM command.")
        else:
            self.armed = False
            response.message = "MOTORS DISARMED (LANDING)"
            self.get_logger().warn("Received DISARM command! Stopping motors.")
        
        response.success = True
        return response

    def publish_sensor_data(self):
        # If disarmed, stop sending data (simulate motors off)
        if not self.armed:
            return 

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Generate noise
        # 5% chance to tilt > 0.5 rad (approx 28 degrees)
        msg.orientation.x = random.uniform(-0.1, 0.1) + (0.6 if random.random() > 0.95 else 0.0)
        msg.orientation.y = random.uniform(-0.1, 0.1)
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneHardware()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()