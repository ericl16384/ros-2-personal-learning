



# this is not a normal node, this is a parent class





import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import AccelStamped

import numpy as np

class CriticallyDampedController(Node):
    def __init__(self, name, w_0):
        super().__init__(name)

        # self.w_0 = w_0
        self.w_0_sq = w_0**2
        self.beta = 2*w_0


        self.control_timestep = 1/60  # 60 Hz
        self.control_timer = self.create_timer(self.control_timestep, self.do_control_step)


        self.relative_pos = np.zeros(3)
        self.relative_vel = np.zeros(3)
        self.relative_accel = np.zeros(3)
        
        self.relative_pos_subscription = self.create_subscription(
            Vector3Stamped, "arm/relative_pos", self.relative_pos_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.relative_vel_subscription = self.create_subscription(
            Vector3Stamped, "arm/relative_vel", self.relative_vel_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.relative_accel_subscription = self.create_subscription(
            Vector3Stamped, "arm/relative_accel", self.relative_accel_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )


        self.acceleration_command = AccelStamped()

        self.accel_command_publisher_publisher = self.create_publisher(
            AccelStamped, "arm/accel_command_publisher",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    def do_control_step(self):

        # a + B*v + w_0**2*x = 0
        # a = - B*v - w_0**2*x
        

        # w_0 = 3              # spring relation (force scaling)
        # B = 2*w_0

        # m = 1                 # todo: change the mass dynamically by measuring the response of the payload/arm system
        # c = 1
        # k = 1


        # operation is not atomic, so let's read these together for concurrency
        relative_pos = self.relative_pos
        relative_vel = self.relative_vel
        relative_accel = self.relative_accel


        accel_command = - relative_accel - relative_vel*self.beta - relative_pos*self.w_0_sq

        self.publish_accel_command(accel_command)

    
    def relative_pos_callback(self, msg:Vector3Stamped):
        self.relative_pos = np.array((msg.vector.x, msg.vector.y, msg.vector.z))
    def relative_vel_callback(self, msg:Vector3Stamped):
        self.relative_vel = np.array((msg.vector.x, msg.vector.y, msg.vector.z))
    def relative_accel_callback(self, msg:Vector3Stamped):
        self.relative_accel = np.array((msg.vector.x, msg.vector.y, msg.vector.z))
    
        
    def publish_accel_command(self, accel_command:np.array):

        current_time_stamp_msg = self.get_clock().now().to_msg()
        frame_id = self.get_namespace().lstrip('/')

        msg = AccelStamped()
        vel_msg.header.stamp = current_time_stamp_msg
        vel_msg.header.frame_id = frame_id
        
        msg.linear.x = accel_command[0]
        msg.linear.y = accel_command[1]
        msg.linear.z = accel_command[2]

        self.arm_head_command_publisher.publish(msg)

