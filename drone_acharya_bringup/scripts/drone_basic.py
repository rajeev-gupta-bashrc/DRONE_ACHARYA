import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

class CustomDroneSubscriber(Node):

    def __init__(self):
        super().__init__('custom_drone_subscriber')

        self.subscription_joint_state = self.create_subscription(
            Point,
            '/world/map/model/custom_drone/joint_state',
            self.joint_state_callback,
            10)
        self.subscription_joint_state  # prevent unused variable warning

        self.subscription_imu = self.create_subscription(
            Imu,
            '/world/map/model/custom_drone/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu',
            self.imu_callback,
            10)
        self.subscription_imu  # prevent unused variable warning

        self.publisher_cmd_force_list = [self.create_publisher(
            Float64, f'/model/custom_drone/joint/iris_with_standoffs::rotor_{index}_joint/cmd_force', 10) for index in range(4)]

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.acceleration_x = 0.0
        self.acceleration_y = 0.0
        self.acceleration_z = 0.0

    def joint_state_callback(self, msg):
        print('getting js')
        pass

    def imu_callback(self, msg):
        print('getting imu')
        pass

    def _publish_command(self, rotor_index, command=0.0):
        cmd_msg = Float64()
        cmd_msg.data = command
        self.publisher_cmd_force[rotor_index].publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    custom_drone_subscriber = CustomDroneSubscriber()
    rclpy.spin(custom_drone_subscriber)
    custom_drone_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
