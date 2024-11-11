#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import transformations as tf_transformations

import argparse
import numpy as np
import time, math

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        self.rotor1_pub = self.create_publisher(Float64, '/drone_v1/rotor_0_joint/cmd_roll', 10)
        self.rotor2_pub = self.create_publisher(Float64, '/drone_v1/rotor_1_joint/cmd_roll', 10)
        self.rotor3_pub = self.create_publisher(Float64, '/drone_v1/rotor_2_joint/cmd_roll', 10)
        self.rotor4_pub = self.create_publisher(Float64, '/drone_v1/rotor_3_joint/cmd_roll', 10)
        
        self.throttle = False
        
        self.command_values = np.zeros(4, dtype=np.float16)
        self.rpy =       np.zeros(3, dtype=np.float32)
        self.w_pqr =     np.zeros(3, dtype=np.float32)          #angular vel
        self.a_xyz =     np.zeros(3, dtype=np.float32)
        self.v_xyz =     np.zeros(3, dtype=np.float32)
        self.posn =      np.zeros(3, dtype=np.float32)
        self.imu_curr_timestamp = np.zeros(2, dtype=np.int64)                 #secs, nsecs
        self.imu_last_timestamp = np.full(2, -1, dtype=np.int64)                 #secs, nsecs
        
        # control algo vars
        self.ctrl_curr_timestamp = np.zeros(2, dtype=np.int64)                 #secs, nsecs
        self.ctrl_last_timestamp = np.full(2, -1, dtype=np.int64)                 #secs, nsecs
        # self.last_posn = np.zeros(3, dtype=np.float32)
        self.control_dict = {
            # 'X': {
            #     'PID': np.array([800.0, 100, 10], dtype=np.float32),
            #     'MAX': 500.0,
            #     'MIN': 420.0,             # (+)hover value - no load = 445
            #     'ERROR': 0.0,
            #     'INTEGRAL': 0.0,
            # },
            # 'Y': {
            #     'PID': np.array([800.0, 100, 10], dtype=np.float32),
            #     'MAX': 500.0,
            #     'MIN': 420.0,             # (+)hover value - no load = 445
            #     'ERROR': 0.0,
            #     'INTEGRAL': 0.0,
            # },
            'Z': {
                'PID': np.array([800.0, 0, 25], dtype=np.float32),
                'MAX': 500.0,
                'MIN': 420.0,             # (+)hover value - no load = 445 or 444.85 (precise)
                'ERROR': 0.0,
                'INTEGRAL': 0.0,
            },
            'ROLL': {
                'PID': np.array([1, 0.001, 0.001], dtype=np.float32),
                'MAX': 5,
                'MIN': -5,
                'ERROR': 0.0,
                'INTEGRAL': 0.0,
            },
            'PITCH': {
                'PID': np.array([1, 0.001, 0.001], dtype=np.float32),
                'MAX': 5,
                'MIN': -5,
                'ERROR': 0.0,
                'INTEGRAL': 0.0,
            },
            'YAW': {
                'PID': np.array([3, 0.1, 400], dtype=np.float32),
                'MAX': 10,
                'MIN': -10,
                'ERROR': 0.0,
                'INTEGRAL': 0.0,
            },
        }
        
        self.desired_posn = np.array([0, 0, 0], dtype=np.float32)
        self.desired_rpy = np.array([0, 0, 0], dtype=np.float32)
        self.logger = self.get_logger()
        self.logger.info('Created Drone Controller Class')
        
        # Change IMU subscriber to Odometry subscriber
        self.odom_sub = None  # Will be initialized in start_odom_subs
        
    def start_controller(self):
        self.throttle = True
        self.timer = self.create_timer(0.010, self.update_controller)
        self.logger.info('Controller started !!')     
        
    def euler_from_quaternion(self, quaternion):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw)
        """
        x, y, z, w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]
        
    def get_rpy(self, rpy_pi):
        ## this was needed to do: 
        ## 1. to get cts angle from rpy
        ## 2. x-axis in the odometry frame is opposite to the global x-axis
        
        rpy = np.zeros(3, dtype=np.float32)
        
        try:
            d_rpy = (rpy_pi - self.last_rpy_pi)
            d_rpy[1] *= -1
            for i in range(3):
                if(d_rpy[i] > np.pi):
                    d_rpy[i] -= 2 * np.pi
                elif(d_rpy[i] < -np.pi):
                    d_rpy[i] += 2 * np.pi
                rpy[i] = self.rpy[i] + d_rpy[i] 
            return rpy
        except Exception as E:
            return rpy
        finally:
            self.last_rpy_pi = rpy_pi
            

    def publish_commands(self):
        if self.throttle:
            self.rotor1_pub.publish(Float64(data=float(self.command_values[0])))
            self.rotor2_pub.publish(Float64(data=float(self.command_values[1])))
            self.rotor3_pub.publish(Float64(data=float(self.command_values[2])))
            self.rotor4_pub.publish(Float64(data=float(self.command_values[3])))
            self.logger.info('Current state: XYZ: (%f, %f, %f)' % (self.posn[0], self.posn[1], self.posn[2]))
            self.logger.info('Current state: RPY: (%f, %f, %f)' % (self.rpy[0], self.rpy[1], self.rpy[2]))
            self.logger.info('Publishing command values: ' + ', '.join(map(str, self.command_values)))
        else:
            self.logger.info('Throttle is unset !!')
            
        
    def start_odom_subs(self):
        self.odom_sub = self.create_subscription(Odometry, '/drone_v1/odometry', self.get_odom, 10)
        self.logger.info('Odometry subscription started!')

    def get_odom(self, msg):
        # Position
        self.posn = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # Orientation (quaternion to euler)
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        rpy_pi = np.array(self.euler_from_quaternion(quat))  
        self.rpy = self.get_rpy(rpy_pi)
        # self.rpy = rpy_pi

        # Linear velocity
        self.v_xyz = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        # Angular velocity
        self.w_pqr = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        # Calculate acceleration from velocity changes
        curr_time = np.array([msg.header.stamp.sec, msg.header.stamp.nanosec])
        if self.imu_last_timestamp[0] > 0:
            del_ts = curr_time - self.imu_last_timestamp
            _t = np.float32(del_ts[0]) + np.float32(del_ts[1]) * 1e-9
            if _t > 0:
                # Calculate acceleration from velocity change
                self.a_xyz = (self.v_xyz - self.last_v_xyz) / _t
        
        # Store current values for next iteration
        self.last_v_xyz = self.v_xyz.copy()
        # self.imu_curr_timestamp = curr_time
        self.imu_last_timestamp = curr_time

    def move_to_height(self, height):
        self.desired_posn[2] = height
        self.logger.info('Desrired Posn set to: ' + ', '.join(map(str, self.desired_posn)))
        
    def bipolar_sigmoid(self, x, max_val, x_for_99_percent_sat = 1):
        return max_val * (2/(1+np.exp(-x * 5.293 / x_for_99_percent_sat)) - 1)
        
    def xy_to_rp(self, del_x, del_y):
        self.max_roll =  0.25 #14.323 944 88 degrees <==> 10m (max)
        self.max_pitch = 0.25 #14.323 944 88 degrees <==> 10m (max)
        del_r = -self.bipolar_sigmoid(del_y, self.max_roll , x_for_99_percent_sat=10)
        del_p = -self.bipolar_sigmoid(del_x, self.max_pitch, x_for_99_percent_sat=10)
        return del_r, del_p
    
    def update_controller(self):
        self.logger.info('\n\nUPDATE CALLED')
        # calc values
        self.ctrl_curr_timestamp = self.imu_last_timestamp
        del_ts = self.ctrl_curr_timestamp - self.ctrl_last_timestamp 
        if del_ts[0] <= 0.0:
            del_ts = [0.010, 0.000]
        _t = np.float32(del_ts[0]) + np.float32(del_ts[1]) * 1e-9
        posn_error = self.desired_posn - self.posn
        rpy_error = self.desired_rpy - self.rpy
        self.logger.info('posn_error: ' + ', '.join(map(str, posn_error)))
        self.logger.info('rpy_error: ' + ', '.join(map(str, rpy_error)))
        
        # rpy error calculation from xy posn error
        del_r, del_p = self.xy_to_rp(posn_error[0], posn_error[1])
        rpy_error[0] += del_r
        rpy_error[1] += del_p
        
        # pwm calculation
        roll_adjust = self.iter_pid('ROLL', rpy_error[0], _t)
        pitch_adjust = self.iter_pid('PITCH', rpy_error[1], _t)
        yaw_adjust = self.iter_pid('YAW', rpy_error[2], _t, verbose=True)
        net_throttle = self.iter_pid('Z', posn_error[2], _t, verbose=True)
        # self.command_values = self.rotor_values_from_throttle(net_throttle, yaw_adjust=yaw_adjust)
        # self.command_values = self.rotor_values_from_throttle(net_throttle, roll_adjust=roll_adjust, pitch_adjust=pitch_adjust, yaw_adjust=yaw_adjust)
        self.command_values = self.rotor_values_from_throttle(net_throttle)
        self.publish_commands()
        self.logger.info('End of update')
        self.logger.info('--------------------------------')
        self.ctrl_last_timestamp = self.ctrl_curr_timestamp
        
    def rotor_values_from_throttle(self, net_throttle=0, roll_adjust=0, pitch_adjust=0, yaw_adjust=0):
        cmd = np.full(4, net_throttle, dtype=np.float16)
        _roll = np.array([-1, 1, 1, -1], dtype=np.float16) * roll_adjust
        _pitch = np.array([-1, 1, -1, 1], dtype=np.float16) * pitch_adjust
        _yaw = np.array([-1, -1, 1, 1], dtype=np.float16) * (yaw_adjust)
        cmd += _roll + _pitch + _yaw
        cmd[2:] *= -1
        return cmd
        
        
    def iter_pid(self, key, error, _t, verbose=False):
        last_error = self.control_dict[key]['ERROR']
        self.control_dict[key]['ERROR'] = error
        self.control_dict[key]['INTEGRAL'] += error * _t
        try:
            P = self.control_dict[key]['PID'][0] * error
            I = self.control_dict[key]['PID'][1] * self.control_dict[key]['INTEGRAL']
            D = self.control_dict[key]['PID'][2] * (error - last_error) / _t
        except Exception as E:
            print(E)
        finally:
            cmd = P + I + D
            for V in [P, I, D]:
                if math.isnan(V):
                    cmd = 0
                    self.logger.info('Nan values detected in PID')
                    # print(_t)
        if cmd > self.control_dict[key]['MAX']:
            cmd = self.control_dict[key]['MAX']
        elif cmd < self.control_dict[key]['MIN']:
            cmd = self.control_dict[key]['MIN']
        if verbose:
            self.logger.info(f'PID for {key}: {P} + {I} + {D} = {cmd}')
        return cmd
            
        

def main(args=None):
    parser = argparse.ArgumentParser(description='Drone Publisher Node')
    parser.add_argument('--takeoff', type=float, default=0.0, help='takeoff height')
    parsed_args = parser.parse_args(args)
    
    rclpy.init(args=args)
    
    drone_node = DroneController()
    drone_node.start_odom_subs()
    drone_node.desired_rpy = np.array([0, 0, 1.57], dtype=np.float32)
    drone_node.move_to_height(parsed_args.takeoff)
    drone_node.start_controller()
    try:
        rclpy.spin(drone_node)
    except KeyboardInterrupt:
        drone_node.logger.info('Keyboard Interrupt')
    finally:
        drone_node.destroy_node()

if __name__ == '__main__':
    main()
