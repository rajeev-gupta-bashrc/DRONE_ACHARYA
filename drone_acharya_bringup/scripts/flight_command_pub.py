#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu


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
        # self.last_rpy  = np.zeros(3, dtype=np.float32)
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
                'PID': np.array([800.0, 100, 10], dtype=np.float32),
                'MAX': 500.0,
                'MIN': 420.0,             # (+)hover value - no load = 445 or 444.85 (precise)
                'ERROR': 0.0,
                'INTEGRAL': 0.0,
            },
            'ROLL': {
                'PID': np.array([0, 0, 0], dtype=np.float32),
                'MAX': 0,
                'MIN': 0,
                'ERROR': 0.0,
                'INTEGRAL': 0.0,
            },
            'PITCH': {
                'PID': np.array([0, 0, 0], dtype=np.float32),
                'MAX': 0,
                'MIN': 0,
                'ERROR': 0.0,
                'INTEGRAL': 0.0,
            },
            'YAW': {
                'PID': np.array([0, 0, 0], dtype=np.float32),
                'MAX': 0,
                'MIN': 0,
                'ERROR': 0.0,
                'INTEGRAL': 0.0,
            },
        }
        
        self.desired_posn = np.array([0, 0, 0], dtype=np.float32)
        self.desired_rpy = np.array([0, 0, 0], dtype=np.float32)
        self.logger = self.get_logger()
        self.logger.info('Created Drone Controller Class')
        
    def start_controller(self):
        self.throttle = True
        self.timer = self.create_timer(0.010, self.update_controller)
        # self.logger.info('Waiting for 5 secs')
        # time.sleep(5)
        self.logger.info('Controller started !!')

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
            
        
    def start_imu_subs(self):
       self.imu_sub = self.create_subscription(Imu, '/drone_v1/imu', self.get_imu, 10)
       self.logger.info('Subscribing started !!')

    def get_imu(self, msg):
        # linear acc
        acc_x = -msg.linear_acceleration.x
        acc_y = -msg.linear_acceleration.y
        acc_z = -msg.linear_acceleration.z - 9.8                    #removing gravity
        # angular vel
        ang_v_x = msg.angular_velocity.x
        ang_v_y = msg.angular_velocity.y
        ang_v_z = msg.angular_velocity.z   
        # rpy     
        orientation = msg.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = np.arctan2(sinp, 1)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        # imu_timestamp
        secs = msg.header.stamp.sec
        nanosecs = msg.header.stamp.nanosec
        self.rpy =              np.array([roll, pitch, yaw])
        self.a_xyz =            np.array([acc_x, acc_y, acc_z])
        self.imu_curr_timestamp =    np.array([secs,      nanosecs])
        self.w_pqr =            np.array([ang_v_x, ang_v_y, ang_v_z])
        # update vel and posn
        del_ts = self.imu_curr_timestamp - self.imu_last_timestamp if self.imu_last_timestamp[0]>0 else [0.001, 0.000]
        _t = np.float32(del_ts[0]) + np.float32(del_ts[1]) * 1e-9
        self.posn += self.v_xyz * _t + 1/2 * self.a_xyz * _t  * _t
        self.v_xyz += self.a_xyz * _t
        self.imu_last_timestamp = self.imu_curr_timestamp
        print(self.a_xyz, self.v_xyz, self.w_pqr, self.rpy)
        
    def move_to_height(self, height):
        self.desired_posn[2] = height
        self.logger.info('Desrired Posn set to: ' + ', '.join(map(str, self.desired_posn)))
        
    def xy_to_rp(self, del_x, dl_y):
        self.max_roll = 0.25 #14.323 944 88 degrees
        self.max_pitch = 0.25 #14.323 944 88 degrees
        del_r = del_x/abs(del_x) * abs(del_x) * self.max_roll / 5
        del_p = del_x/abs(del_y) * abs(del_y) * self.max_roll / 5
    def update_controller(self):
        # calc values
        self.logger.info('update called')
        self.ctrl_curr_timestamp = self.imu_curr_timestamp
        del_ts = self.ctrl_curr_timestamp - self.ctrl_last_timestamp if self.ctrl_last_timestamp[0] > 0 else [0.010, 0.000]
        _t = np.float32(del_ts[0]) + np.float32(del_ts[1]) * 1e-9
        posn_error = self.desired_posn - self.posn
        print('posn_error', posn_error)
        net_throttle = self.iter_pid('Z', posn_error[2], _t, verbose=True)
        self.desired_rpy[0] = posn_error[0]
        rpy_error = self.desired_rpy - self.rpy
        roll_adjust = self.iter_pid('ROLL', rpy_error[0], _t)
        pitch_adjust = self.iter_pid('PITCH', rpy_error[1], _t)
        yaw_adjust = self.iter_pid('YAW', rpy_error[2], _t)
        # self.command_values = self.rotor_values_from_throttle(net_throttle, roll_adjust=roll_adjust, yaw_adjust=yaw_adjust)
        self.command_values = self.rotor_values_from_throttle(net_throttle)
        self.publish_commands()
        self.ctrl_last_timestamp = self.ctrl_curr_timestamp
        
    def rotor_values_from_throttle(self, net_throttle=0, roll_adjust=0, pitch_adjust=0, yaw_adjust=0):
        cmd = np.full(4, net_throttle, dtype=np.float16)
        _roll = np.array([1, -1, -1, 1], dtype=np.float16) * roll_adjust
        _pitch = np.array([1, -1, 1, -1], dtype=np.float16) * pitch_adjust
        _yaw = np.array([-1, -1, 1, 1], dtype=np.float16) * yaw_adjust
        # print(cmd, _roll, _pitch, _yaw)
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
                    break
        if verbose:
            self.logger.info(f'PID for {key}: {P} + {I} + {D} = {cmd}')
        if cmd > self.control_dict[key]['MAX']:
            return self.control_dict[key]['MAX']
        elif self.control_dict[key]['MIN']:
            return self.control_dict[key]['MIN']
        else:
            return cmd
            
        

def main(args=None):
    parser = argparse.ArgumentParser(description='Drone Publisher Node')
    parser.add_argument('--takeoff', type=float, default=0.0, help='takeoff height')
    parsed_args = parser.parse_args(args)
    
    rclpy.init(args=args)
    
    drone_node = DroneController()
    drone_node.start_imu_subs()
    drone_node.move_to_height(parsed_args.takeoff)
    drone_node.start_controller()
    
    rclpy.spin(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
