import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker  # <--- NEW
import random
import math

class DroneHardware(Node):
    def __init__(self):
        super().__init__('drone_hardware')

        
        # 1. The Sensor Publisher (Best Effort for sensors)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(Imu, "imu", qos_profile)
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        
        # 2. The Visualizer Publisher (New)
        # We publish to '~/visual_vector' which becomes '/alpha/hardware_node/visual_vector'
        self.vis_pub = self.create_publisher(Marker, 'visual_vector', 10)

        # 3. The Safety Service
        self.srv = self.create_service(SetBool, 'arm_disarm', self.set_armed_callback)

        # State Variable
        self.armed = True
        
        self.get_logger().info("Hardware Node Started (Phase 3 Visuals Active)")

    def set_armed_callback(self, request, response):
        self.armed = request.data
        response.success = True
        response.message = f"Armed status set to: {self.armed}"
        if not self.armed:
            self.get_logger().warn("DISARM REQUEST RECEIVED. MOTORS STOPPED.")
        else:
            self.get_logger().info("System Re-Armed.")
        return response

    def publish_sensor_data(self):
        if not self.armed:
            return

        # --- SIMULATE SENSOR DATA ---
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Important: Frame ID must match the TF child frame (e.g., 'alpha')
        # We use the namespace to guess the frame name, or hardcode generic 'base_link'
        # if we were using a full URDF. For now, we will assume the namespace IS the frame.
        msg.header.frame_id = self.get_namespace().lstrip('/')
        # msg.header.frame_id = 'base_link'

        # Generate noise
        # 2.5% chance to tilt > 0.5 rad (approx 28 degrees)
        msg.orientation.x = random.uniform(-0.1, 0.1) + (0.6 if random.random() > 0.975 else 0.0)
        msg.orientation.y = random.uniform(-0.1, 0.1)

        # Simulate gravity (9.81 m/s^2) + noise
        noise = random.uniform(-0.5, 0.5)
        msg.linear_acceleration.z = -9.81 + noise
        msg.linear_acceleration.x = noise
        msg.linear_acceleration.y = noise

        self.publisher_.publish(msg)
        
        # --- PUBLISH VISUALIZATION ---
        self.publish_visual_marker(msg)

    def publish_visual_marker(self, imu_msg):
        marker = Marker()
        marker.header.frame_id = imu_msg.header.frame_id # Stick the arrow to the drone
        marker.header.stamp = imu_msg.header.stamp
        
        marker.ns = "gravity_vector"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Scale the arrow (Thin and long)
        marker.scale.x = 0.05 # Shaft diameter
        marker.scale.y = 0.1  # Head diameter
        marker.scale.z = 0.5  # Head length
        
        # Color (Red and Opaque)
        marker.color.r = 1.0
        marker.color.a = 1.0

        # Start point (0,0,0 relative to the drone)
        # End point (Where the force is pointing)
        # We visualize the acceleration vector scaled down by 0.1 so it fits
        from geometry_msgs.msg import Point
        start = Point() # 0,0,0
        end = Point()
        end.x = imu_msg.linear_acceleration.x * 0.1
        end.y = imu_msg.linear_acceleration.y * 0.1
        end.z = imu_msg.linear_acceleration.z * 0.1
        
        marker.points = [start, end]
        
        self.vis_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DroneHardware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()