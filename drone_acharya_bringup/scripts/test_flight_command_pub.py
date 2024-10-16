#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Float64
import argparse

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf2_ros
import numpy as np
from collections import deque



class DronePublisher(Node):
    def __init__(self, del_throttle, throttle):
        super().__init__('drone_publisher')
        
        self.rotor1_pub = self.create_publisher(Float64, '/drone_v1/rotor_0_joint/cmd_roll', 10)
        self.rotor2_pub = self.create_publisher(Float64, '/drone_v1/rotor_1_joint/cmd_roll', 10)
        self.rotor3_pub = self.create_publisher(Float64, '/drone_v1/rotor_2_joint/cmd_roll', 10)
        self.rotor4_pub = self.create_publisher(Float64, '/drone_v1/rotor_3_joint/cmd_roll', 10)
        
        self.timer = self.create_timer(1.0, self.publish_commands)
        
        self.command_values = {
            'rotor_0_joint':  throttle+del_throttle,
            'rotor_1_joint':  throttle+del_throttle,
            'rotor_2_joint': -throttle,
            'rotor_3_joint': -throttle
        }
        
        self.x = self.y = self.z = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.wx = self.wy = self.wz = 0.0
    
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.last_secs_nsecs = [0, 0]
        self.last_wz = 0.0
        self.alpha_z = 0.0
        
        self.queue_len = 10
        self.queue_alpha = deque(maxlen=self.queue_len)
        self.counter = 0
        
        
        
    def start_odometry(self):
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/drone_v2/odometry',
            self.odometry_callback,
            10)
    
    def odometry_callback(self, msg):
        # Position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        # Orientation
        orientation_q = msg.pose.pose.orientation
        self.update_rpy(orientation_q)

        # Linear velocities
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z

        # Angular velocities
        self.wx = msg.twist.twist.angular.x
        self.wy = msg.twist.twist.angular.y
        self.wz = msg.twist.twist.angular.z
        
        curr_time = self.get_clock().now()
        del_t = curr_time.seconds_nanoseconds()[0]-self.last_secs_nsecs[0] + 1e-9 * (curr_time.seconds_nanoseconds()[1]-self.last_secs_nsecs[1])
        # print('time: ', curr_time.seconds_nanoseconds())
        del_wz = self.wz - self.last_wz
        try:
            self.alpha_z = del_wz/del_t
        except ZeroDivisionError:
            print('Zero Division Error')
            self.alpha_z = 0.0
        self.last_secs_nsecs[0], self.last_secs_nsecs[1] = curr_time.seconds_nanoseconds()[0], curr_time.seconds_nanoseconds()[1]
        self.last_wz = self.wz
        # print('del_wz: ', del_wz, 'del_t: ', del_t, 'omega about z: ', self.wz, 'alpha_z: ', self.alpha_z)
        if self.alpha_z!=0.0: 
            self.queue_alpha.append(self.alpha_z)
            self.counter+=1
            if self.counter % self.queue_len == 0:
                print('alpha_z: ', sum(self.queue_alpha)/self.queue_len)
                self.counter = 0

    def update_rpy(self, orientation_q):
        # Convert quaternion to Euler angles
        try:
            dummy_transform = self.tf_buffer.lookup_transform('dummy', 'dummy', rclpy.time.Time())
            dummy_transform.transform.rotation = orientation_q
            (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion(dummy_transform.transform.rotation)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Failed to convert quaternion to Euler angles")

    def euler_from_quaternion(self, quaternion):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw)
        """
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
        
    def publish_commands(self):
        self.rotor1_pub.publish(Float64(data=self.command_values['rotor_0_joint']))
        self.rotor2_pub.publish(Float64(data=self.command_values['rotor_1_joint']))
        self.rotor3_pub.publish(Float64(data=self.command_values['rotor_2_joint']))
        self.rotor4_pub.publish(Float64(data=self.command_values['rotor_3_joint']))
        self.get_logger().info('Publishing command values: %s' % self.command_values)

def main(args=None):
    parser = argparse.ArgumentParser(description='Drone Publisher Node')
    parser.add_argument('--throttle', type=float, default=100.0, help='Throttle value')
    parser.add_argument('--del_throttle', type=float, default=10.0, help='Delta Throttle value')
    parsed_args = parser.parse_args(args)
    
    rclpy.init(args=args)
    
    node = DronePublisher(del_throttle=parsed_args.del_throttle, throttle=parsed_args.throttle)
    node.start_odometry()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
## Observations:
# For the calculations of Cq (torque coefficient):
# 1. 
    # alpha_z:  -0.13714849146831487
    # alpha_z:  -0.13626797042643093
    # alpha_z:  -0.13553397346032264
    # alpha_z:  -0.13596785531627065
    # [INFO] [1727129564.202014901] [drone_publisher]: Publishing command values: {'rotor_0_joint': 469.0, 'rotor_1_joint': 469.0, 'rotor_2_joint': -419.0, 'rotor_3_joint': -419.0}

    # Cq = Iz * alpha_z / [omega1**2 + omega2**2 - omega3**2 - omega4**2] 
    # Cq = 0.00000002608733668


# 2. 
    # alpha_z:  -0.032295702367438395
    # alpha_z:  -0.032471520540381604
    # alpha_z:  -0.0329908367688448
    # alpha_z:  -0.03220345666288479
    # [INFO] [1727130365.845835301] [drone_publisher]: Publishing command values: {'rotor_0_joint': 225.0, 'rotor_1_joint': 225.0, 'rotor_2_joint': -200.0, 'rotor_3_joint': -200.0}
    # Cq = 0.00000002654640694