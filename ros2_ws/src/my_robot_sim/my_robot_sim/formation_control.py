import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        
        # Create publishers for simulated robots
        self.pub_r1 = self.create_publisher(Twist, '/robot1/mecanum_cont/cmd_vel_unstamped', 10)
        self.pub_r2 = self.create_publisher(Twist, '/robot2/mecanum_cont/cmd_vel_unstamped', 10)
        
        # Control loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        # Robot 1: Strafing logic (Mecanum unique ability)
        twist1 = Twist()
        twist1.linear.x = 0.5  # Forward
        twist1.linear.y = 0.5  # Slide Left
        
        # Robot 2: Spinning logic
        twist2 = Twist()
        twist2.linear.x = 0.0
        twist2.angular.z = 1.0 # Rotate in place

        self.pub_r1.publish(twist1)
        self.pub_r2.publish(twist2)

def main(args=None):
    rclpy.init(args=args)
    node = FormationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()