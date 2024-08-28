#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class DronePublisher(Node):
    def __init__(self):
        super().__init__('drone_publisher')
        
        self.rotor1_pub = self.create_publisher(Float64, '/drone_v1/rotor_0_joint/cmd_roll', 10)
        self.rotor2_pub = self.create_publisher(Float64, '/drone_v1/rotor_1_joint/cmd_roll', 10)
        self.rotor3_pub = self.create_publisher(Float64, '/drone_v1/rotor_2_joint/cmd_roll', 10)
        self.rotor4_pub = self.create_publisher(Float64, '/drone_v1/rotor_3_joint/cmd_roll', 10)
        
        self.timer = self.create_timer(1.0, self.publish_commands)
        
        self.command_values = {
            'rotor_0_joint': 100.0,
            'rotor_1_joint': 100.0,
            'rotor_2_joint': 100.0,
            'rotor_3_joint': 100.0
        }
        
    def publish_commands(self):
        self.rotor1_pub.publish(Float64(data=self.command_values['rotor_0_joint']))
        self.rotor2_pub.publish(Float64(data=self.command_values['rotor_1_joint']))
        self.rotor3_pub.publish(Float64(data=self.command_values['rotor_2_joint']))
        self.rotor4_pub.publish(Float64(data=self.command_values['rotor_3_joint']))
        self.get_logger().info('Publishing command values: %s' % self.command_values)

def main(args=None):
    rclpy.init(args=args)
    node = DronePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('******** interrupted *********')
        # for key, val in node.command_values.items():
        #     node.command_values[key] = 0.0
        # node.publish_commands()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
