





# COPIED FROM arm_controller.py

# because I'm not very good with nodes yet






import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Accel
from std_srvs.srv import SetBool # <--- Service Type

import numpy as np

class DrivetrainController(Node):
    def __init__(self):
        super().__init__("drivetrain_controller")

        self.control_timestep = 1/60  # 60 Hz
        self.control_timer = self.create_timer(self.control_timestep, self.do_control_step)


        self.last_r = None
        self.last_v = None
        self.target_position = None
        
        self.drivetrain_pose_subscription = self.create_subscription(
            PoseStamped, "drivetrain_pose_stamped", self.drivetrain_pose_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.drivetrain_vel_subscription = self.create_subscription(
            Vector3Stamped, "drivetrain_vel_stamped", self.drivetrain_vel_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.drivetrain_target_pos_subscription = self.create_subscription(
            Vector3Stamped, "drivetrain_target_pos", self.drivetrain_target_pos_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )


        self.acceleration_command = Accel()

        self.drivetrain_command_publisher = self.create_publisher(
            Accel, "drivetrain_command_accel",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    def do_control_step(self):

        # a + B*v + w_0**2*x = 0
        # a = -( B*v + w_0**2*x )
        

        w_0 = 3              # spring relation (force scaling)
        B = 2*w_0

        # m = 1                 # todo: change the mass dynamically by measuring the response of the payload/arm system
        # c = 1
        # k = 1


        # operation is not atomic, so let's read these together for concurrency
        r = self.last_r
        v = self.last_v
        target_position = self.target_position

        for param in (r, v, target_position):
            if isinstance(param, type(None)):
                self.get_logger().debug(f"control loop disabled; inputs not set")
                return


        # position = np.array((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        displacement = r - target_position
        velocity = v
        acceleration = - velocity*B - displacement*w_0**2


        # self.get_logger().info(f"{displacement}")


        self.acceleration_command.linear.x = acceleration[0]
        self.acceleration_command.linear.y = acceleration[1]
        self.acceleration_command.linear.z = acceleration[2]

        self.publish_accel_command()

    
    def drivetrain_pose_callback(self, msg):
        self.last_r = np.array((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    
    def drivetrain_vel_callback(self, msg):
        self.last_v = np.array((msg.vector.x, msg.vector.y, msg.vector.z))
    
    def drivetrain_target_pos_callback(self, msg):
        self.target_position = np.array((msg.vector.x, msg.vector.y, msg.vector.z))

        self.get_logger().info(f"arm target pos set: {self.target_position}")
        
    def publish_accel_command(self):
        msg = Accel()
        
        msg.linear.x = self.acceleration_command.linear.x
        msg.linear.y = self.acceleration_command.linear.y
        msg.linear.z = self.acceleration_command.linear.z

        self.drivetrain_command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrivetrainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


    