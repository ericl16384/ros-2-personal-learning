import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
# from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Accel
from std_srvs.srv import SetBool # <--- Service Type

class ArmHardwareSimulator(Node):
    def __init__(self):
        super().__init__("arm_hardware_simulator")


        self.pose = Pose()
        self.velocity = Vector3()
        self.acceleration = Accel()

        # self.pose.position.z = 1

        self.physics_timestep = 0.1

        self.arm_head_physics_timer = self.create_timer(self.physics_timestep, self.do_euler_integration_physics)

        self.arm_head_pose_publisher = self.create_publisher(
            PoseStamped, "arm_head_pose_stamped",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.arm_head_command_subscription = self.create_subscription(
            Accel, "arm_head_command_accel", self.head_command_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
    
    def do_euler_integration_physics(self):
        
        # self.get_logger().info(f"euler integration")

        self.velocity.x += self.acceleration.linear.x * self.physics_timestep
        self.velocity.y += self.acceleration.linear.y * self.physics_timestep
        self.velocity.z += self.acceleration.linear.z * self.physics_timestep

        self.pose.position.x += self.velocity.x * self.physics_timestep
        self.pose.position.y += self.velocity.y * self.physics_timestep
        self.pose.position.z += self.velocity.z * self.physics_timestep

        self.publish_sensor_data()
    
    def publish_sensor_data(self):
        
        self.get_logger().info(f"publishing position: {self.pose.position.x} {self.pose.position.y} {self.pose.position.z}")

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.header.frame_id = self.get_namespace().lstrip('/')
        msg.pose.position.x = self.pose.position.x
        msg.pose.position.y = self.pose.position.y
        msg.pose.position.z = self.pose.position.z

        self.arm_head_pose_publisher.publish(msg)
        # self.publish_visual_marker(msg)
    
    # def apply_acceleration(self):
        
    #     # self.get_logger().info("apply_acceleration_callback ")
    #     pass

    def head_command_callback(self, msg):
        self.acceleration = msg
        
        self.get_logger().info(f"updated acceleration command: {self.acceleration}")



def main(args=None):
    rclpy.init(args=args)
    node = ArmHardwareSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
