import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool # <--- Service Type

class NetworkedFlightComputer(Node):
    def __init__(self):
        super().__init__("networked_flight_computer")
        
        # Subscriber Setup (Must match Publisher QoS)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Imu, "drone/imu", self.imu_callback, qos_profile)
        
        # SERVICE CLIENT: Connects to the server
        self.landing_client = self.create_client(SetBool, "drone/set_armed")
        
        # Flag to prevent spamming the kill switch
        self.emergency_triggered = False

        # Wait for hardware to come online
        while not self.landing_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for hardware service...")

    def imu_callback(self, msg):
        if self.emergency_triggered:
            return # Stop processing if we already triggered the kill switch

        current_roll = abs(msg.orientation.x)
        
        # Check for dangerous tilt
        if current_roll > 0.5:
            self.get_logger().error(f"CRITICAL TILT ({current_roll:.2f}). SENDING KILL COMMAND!")
            self.trigger_landing()
        else:
            self.get_logger().info(f"Status Normal: Roll {current_roll:.2f}")

    def trigger_landing(self):
        self.emergency_triggered = True
        
        # Create the request object
        req = SetBool.Request()
        req.data = False # False = Disarm/Land
        
        # Send asynchronously (non-blocking)
        self.future = self.landing_client.call_async(req)
        # Define what happens when the hardware replies
        self.future.add_done_callback(self.landing_response_callback)

    def landing_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Hardware replied: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NetworkedFlightComputer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()