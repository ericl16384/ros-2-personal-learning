import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

class FlightComputer(Node):
    def __init__(self):
        super().__init__("flight_computer")
        
        # Must match the Publisher's QoS "Best Effort" or they won't connect!
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.subscription = self.create_subscription(
            Imu,
            "/drone/imu",
            self.imu_callback,
            qos_profile)
            
    def imu_callback(self, msg):
        # Check roll (x) orientation
        current_roll = abs(msg.orientation.x)
        
        if current_roll > 0.5: # Approx 28 degrees
            self.get_logger().warn(f"CRITICAL: TILT DETECTED ({current_roll:.2f} rad). TRIGGERING LANDING!")
            # Here is where you would call the "Land" service in Phase 2
        else:
            self.get_logger().info(f"Status Normal: Roll {current_roll:.2f}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = FlightComputer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()