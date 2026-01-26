import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
# from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Accel

class DrivetrainHardwareSimulator(Node):
    def __init__(self):
        super().__init__("drivetrain_hardware_simulator")


        self.pose = Pose()
        self.velocity = Vector3()

        self.pose.position.x = 1
        self.pose.position.y = 2

        self.acceleration = Accel()

        # self.pose.position.z = 1

        self.physics_timestep = 0.01
        self.drivetrain_physics_timer = self.create_timer(self.physics_timestep, self.do_euler_integration_physics)

        self.drivetrain_pose_publisher = self.create_publisher(
            PoseStamped, "drivetrain_pose_stamped",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.drivetrain_vel_publisher = self.create_publisher(
            Vector3Stamped, "drivetrain_vel_stamped",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.drivetrain_command_subscription = self.create_subscription(
            Accel, "drivetrain_command_accel", self.drivetrain_command_callback,
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
        
        # self.get_logger().info(f"publishing position: {self.pose.position.x} {self.pose.position.y} {self.pose.position.z}")

        current_time_stamp_msg = self.get_clock().now().to_msg()
        frame_id = self.get_namespace().lstrip('/')


        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time_stamp_msg
        pose_msg.header.frame_id = frame_id
        
        pose_msg.pose.position.x = self.pose.position.x
        pose_msg.pose.position.y = self.pose.position.y
        pose_msg.pose.position.z = self.pose.position.z

        self.drivetrain_pose_publisher.publish(pose_msg)


        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = current_time_stamp_msg
        vel_msg.header.frame_id = frame_id

        vel_msg.vector.x = self.velocity.x
        vel_msg.vector.y = self.velocity.y
        vel_msg.vector.z = self.velocity.z

        self.drivetrain_vel_publisher.publish(vel_msg)

        # self.publish_visual_marker(msg)
    
    # def apply_acceleration(self):
        
    #     # self.get_logger().info("apply_acceleration_callback ")
    #     pass

    def drivetrain_command_callback(self, msg):
        self.acceleration = msg
        
        # self.get_logger().info(f"updated acceleration command: {self.acceleration.linear}")



def main(args=None):
    rclpy.init(args=args)
    node = DrivetrainHardwareSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


