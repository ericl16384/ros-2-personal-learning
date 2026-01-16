import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointReader(Node):
    def __init__(self):
        super().__init__('joint_reader_node')
        
        # Best Practice: Declare a parameter for the target joint name
        # rather than hardcoding it.
        self.declare_parameter('target_joint', 'left_wheel')
        self.target_joint_name = self.get_parameter('target_joint').get_parameter_value().string_value

        # Subscribe to the topic published by the Gazebo plugin
        # The standard topic is '/joint_states'
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10  # QoS History depth
        )
        self.get_logger().info(f'Listening for joint: {self.target_joint_name}')

    def listener_callback(self, msg):
        try:
            # 1. Find the index of the joint we care about
            if self.target_joint_name in msg.name:
                index = msg.name.index(self.target_joint_name)
                
                # 2. Access the data at that index
                current_position = msg.position[index]
                current_velocity = msg.velocity[index] if len(msg.velocity) > index else 0.0
                
                self.get_logger().info(
                    f'Joint "{self.target_joint_name}" - '
                    f'Position: {current_position:.4f} rad, '
                    f'Velocity: {current_velocity:.4f} rad/s'
                )
            else:
                # Useful for debugging if you spelled the name wrong in the param
                self.get_logger().debug(f'Joint {self.target_joint_name} not found in message.')
                
        except ValueError as e:
            self.get_logger().error(f'Error processing joint state: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JointReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()