from import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
# from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Accel
from std_srvs.srv import SetBool # <--- Service Type

class ArmHardwareSimulator(Node):
    def __init__(self):
        super().__init__("arm-hardware-simulator")


        self.position = Pose()


        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.arm_head_pose_publisher = self.create_publisher(PoseStamped, "arm_head_pose_stamped", qos_profile)

        self.arm_head_pose_publisher_timer = self.create_timer(0.1, self.publish_sensor_data)
        
        self.srv = self.create_service(SetBool, 'arm_disarm', self.set_armed_callback)
    
    def publish_sensor_data(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.header.frame_id = self.get_namespace().lstrip('/')
        msg.position.x = self.position.x
        msg.position.y = self.position.y
        msg.position.z = self.position.z

        self.arm_head_pose_publisher.publish(msg)
        self.publish_visual_marker(msg)
    
    def apply_acceleration_callback(self):
        
        self.get_logger().info("apply_acceleration_callback triggered")


def main():
    rclpy.init()
    minimal_service = ArmHardwareSimulator()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    