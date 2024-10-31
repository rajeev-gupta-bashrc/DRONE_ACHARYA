#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/drone_v1/odometry',
            self.listener_callback,
            10)
        self.subscription
        self.latest_message = None
        self.ros_clock = Clock(clock_type=ClockType.ROS_TIME)
        self.node_time = self.ros_clock.now()  


    def listener_callback(self, msg):
        self.latest_message = msg
        self.get_logger().info(f'Height: {msg.pose.pose.position.z}')

    def get_current_message(self):
        return self.latest_message


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    # Create a rate object (e.g., 2 Hz or 2 times per second)
    # rate = minimal_subscriber.create_rate(2)  # 2 Hz

    try:
        while rclpy.ok():
            # Use spin_once to process callbacks
            minimal_subscriber.get_logger().info('Initiating to read')
            # rclpy.spin_once(minimal_subscriber, timeout_sec=0.1)
            rclpy.spin_once(minimal_subscriber)

            # Get the current message, if any
            current_msg = minimal_subscriber.get_current_message()
            minimal_subscriber.get_logger().info('Got the message')
            if current_msg:
                minimal_subscriber.get_logger().info(f'Current message: {current_msg.pose.pose.position.z}')
            
            # Sleep to maintain the loop at the desired rate (2 Hz)
            
            minimal_subscriber.get_logger().info('Going to sleep')
            while(minimal_subscriber.ros_clock.now().seconds_nanoseconds()[0] != minimal_subscriber.node_time.seconds_nanoseconds()[0] + 5):
                continue
            else:
                
                minimal_subscriber.node_time = minimal_subscriber.ros_clock.now()
            minimal_subscriber.get_logger().info('Slept enough, end of loop\n\n')

    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
