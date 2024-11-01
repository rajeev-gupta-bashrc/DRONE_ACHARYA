#!/usr/bin/env python3
import rclpy
import rclpy.client
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np

import argparse
import numpy as np
import time, math, os
import sys 
import logging


from rclpy.clock import Clock, ClockType

pkg_path = '/home/rajeev-gupta/ros2/btp_ws/src/DRONE_ACHARYA/drone_acharya_bringup'
sys.path.append(pkg_path + '/scripts')
from support_files_drone import SupportFilesDrone as sfd

class RosRate():
    def __init__(self, frequency: float, clock: Clock):
        self.frequency = frequency
        self.Time_period = 1/self.frequency
        self.T_secs = int(self.Time_period)
        self.T_nsecs = (self.Time_period - self.T_secs) * 1e9
        self.clock = clock
        self.curr_time = self.clock.now()
        
    def sleep(self):
        self.curr_time = self.clock.now()
        while(self.clock.now().seconds_nanoseconds()[0] != self.curr_time.seconds_nanoseconds()[0] + self.T_secs and \
              self.clock.now().seconds_nanoseconds()[1] != self.curr_time.seconds_nanoseconds()[1] + self.T_nsecs):
            continue

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        self.rotor1_pub = self.create_publisher(Float64, '/drone_v1/rotor_0_joint/cmd_roll', 10)
        self.rotor2_pub = self.create_publisher(Float64, '/drone_v1/rotor_1_joint/cmd_roll', 10)
        self.rotor3_pub = self.create_publisher(Float64, '/drone_v1/rotor_2_joint/cmd_roll', 10)
        self.rotor4_pub = self.create_publisher(Float64, '/drone_v1/rotor_3_joint/cmd_roll', 10)
        
        self.throttle = False
        
        self.omega_values = np.zeros(4, dtype=np.float16)
        self.x = self.y = self.z = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.wx = self.wy = self.wz = 0.0
        self.states = np.array([self.vx, self.vy, self.vz, self.wx, self.wy, self.wz, self.x, self.y, self.z, self.roll, self.pitch, self.yaw])
        
        
        self.desired_posn = np.array([0, 0, 0], dtype=np.float32)
        self.desired_vel = np.array([0, 0, 0], dtype=np.float32)
        self.desired_acc = np.array([0, 0, 0], dtype=np.float32)
        self.desired_psi = 0.0
        self.logger = self.get_logger()
        self.logger.info('Created Drone Controller Class')
        
        self.state_trajectory = np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.simulation_clock = Clock(clock_type=ClockType.ROS_TIME)
        self.controller_update_rate = RosRate(10, self.simulation_clock)
        
        self.log_file_name = 'logs/ros2_log.txt'
        if os.path.exists(pkg_path + self.log_file_name):
            self.log_file_name += '_1'
            while os.path.exists(pkg_path + self.log_file_name):
                self.log_file_name = self.log_file_name[:-1] + str(int(self.log_file_name[-1])+1)
        logging.basicConfig(filename = pkg_path + self.log_file_name, level=logging.INFO)

        
        
    def start_odometry(self):
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/drone_v1/odometry',
            self.odometry_callback,
            10)
        
    def init_mpc(self):
        self.throttle = True
        # Create an object for the support functions.
        self.support = sfd()
        self.constants = self.support.constants

        # Load the constant values needed in the main file
        # Ts = constants["Ts"]
        self.controlled_states = self.constants["controlled_states"]  # number of outputs
        self.innerDyn_length = self.constants[
            "innerDyn_length"
        ]  # number of inner control loop iterations
        pos_x_y = self.constants["pos_x_y"]
        if pos_x_y == 1:
            extension = 2.5
        elif pos_x_y == 0:
            extension = 0
        else:
            self.logger.info(
                "Please make pos_x_y variable either 1 or 0 in the support file initial funtcion (where all the initial constants are)"
            )
            exit()
        self.sub_loop = self.constants["sub_loop"]
        
        # Initial drone propeller states
        self.omega_values[0]= 110 * np.pi / 3  # rad/s
        self.omega_values[1]= 110 * np.pi / 3  # rad/s
        self.omega_values[2]= 110 * np.pi / 3  # rad/s
        self.omega_values[3]= 110 * np.pi / 3  # rad/s
        self.omega_total = sum(self.omega_values)

        self.ct = self.constants["ct"]
        self.cq = self.constants["cq"]
        self.l =  self.constants["l"]
        
        self.U1 = self.ct * (self.omega_values[0]**2 + self.omega_values[1]**2 + self.omega_values[2]**2 + self.omega_values[3]**2)
        self.U2 = self.ct * self.l * (self.omega_values[1]**2 - self.omega_values[3]**2)
        self.U3 = self.ct * self.l * (self.omega_values[2]**2 - self.omega_values[0]**2)
        self.U4 = self.cq * (-(self.omega_values[0]**2) + self.omega_values[1]**2 - self.omega_values[2]**2 + self.omega_values[3]**2)
        rclpy.spin_once(node=self, timeout_sec=self.controller_update_rate.Time_period)
        

        
    def go_to_goal(self, X_ref, X_dot_ref, X_dot_dot_ref, 
                         Y_ref, Y_dot_ref, Y_dot_dot_ref,
                         Z_ref, Z_dot_ref, Z_dot_dot_ref,
                         psi_ref):
        # update odometry
        rclpy.spin_once(node=self, timeout_sec=self.controller_update_rate.Time_period)
        
        # Implement the position controller (state feedback linearization)
        phi_ref, theta_ref, self.U1 = self.support.pos_controller(
            X_ref,
            X_dot_ref,
            X_dot_dot_ref,
            Y_ref,
            Y_dot_ref,
            Y_dot_dot_ref,
            Z_ref,
            Z_dot_ref,
            Z_dot_dot_ref,
            psi_ref,
            self.states,
        )
        Phi_ref = np.transpose([phi_ref * np.ones(self.innerDyn_length + 1)])
        Theta_ref = np.transpose([theta_ref * np.ones(self.innerDyn_length + 1)])

        # Make Psi_ref increase continuosly in a linear fashion per outer loop
        Psi_ref = np.transpose([np.zeros(self.innerDyn_length + 1)])
        # for yaw_step in range(0, self.innerDyn_length + 1):
        #     Psi_ref[yaw_step] = (
        #         psi_ref[i_global]
        #         + (psi_ref[i_global + 1] - psi_ref[i_global])
        #         / (Ts * self.innerDyn_length)
        #         * Ts
        #         * yaw_step
        #     )

        temp_angles = np.concatenate(
            (
                Phi_ref[1 : len(Phi_ref)],
                Theta_ref[1 : len(Theta_ref)],
                Psi_ref[1 : len(Psi_ref)],
            ),
            axis=1,
        )
        # Create a reference vector
        refSignals = np.zeros(len(Phi_ref) * self.controlled_states)

        # Build up the reference signal vector:
        # refSignal = [Phi_ref_0, Theta_ref_0, Psi_ref_0, Phi_ref_1, Theta_ref_2, Psi_ref_2, ... etc.]
        k = 0
        for i in range(0, len(refSignals), self.controlled_states):
            refSignals[i] = Phi_ref[k]
            refSignals[i + 1] = Theta_ref[k]
            refSignals[i + 2] = Psi_ref[k]
            k = k + 1

        # Initiate the controller - simulation loops
        hz = self.support.constants["hz"]  # horizon period
        k = 0  # for reading reference signals
        for i in range(0, self.innerDyn_length):
            self.logger.info('******* InnerDyn Running ******* %i th iteration' % i)
            # update odometry
            rclpy.spin_once(node=self, timeout_sec=self.controller_update_rate.Time_period)
            
            # Generate the discrete state space matrices
            (
                Ad,
                Bd,
                Cd,
                Dd,
                x_dot,
                y_dot,
                z_dot,
                phi,
                phi_dot,
                theta,
                theta_dot,
                psi,
                psi_dot,
            ) = self.support.LPV_cont_discrete(self.states, self.omega_total)
            x_dot = np.transpose([x_dot])
            y_dot = np.transpose([y_dot])
            z_dot = np.transpose([z_dot])
            # Generate the augmented current state and the reference vector
            x_aug_t = np.transpose(
                [
                    [phi, phi_dot, theta, theta_dot, psi, psi_dot, self.U2, self.U3, self.U4],
                ]
            )
            # Ts=0.1 s
            # From the refSignals vector, only extract the reference values from your [current sample (NOW) + Ts] to [NOW+horizon period (hz)]
            # Example: t_now is 3 seconds, hz = 15 samples, so from refSignals vectors, you move the elements to vector r:
            # r=[Phi_ref_3.1, Theta_ref_3.1, Psi_ref_3.1, Phi_ref_3.2, ... , Phi_ref_4.5, Theta_ref_4.5, Psi_ref_4.5]
            # With each loop, it all shifts by 0.1 second because Ts=0.1 s
            k = k + self.controlled_states
            if k + self.controlled_states * hz <= len(refSignals):
                r = refSignals[k : k + self.controlled_states * hz]
            else:
                r = refSignals[k : len(refSignals)]
                hz = hz - 1

            # Generate the compact simplification matrices for the cost function
            Hdb, Fdbt, Cdb, Adc = self.support.mpc_simplification(Ad, Bd, Cd, Dd, hz)
            ft = np.matmul(
                np.concatenate((np.transpose(x_aug_t)[0][0 : len(x_aug_t)], r), axis=0),
                Fdbt,
            )

            du = -np.matmul(np.linalg.inv(Hdb), np.transpose([ft]))

            # Update the real inputs
            self.U2 = self.U2 + du[0][0]
            self.U3 = self.U3 + du[1][0]
            self.U4 = self.U4 + du[2][0]

            # Compute the new omegas based on the new U-s
            U1C = self.U1 /  self.ct
            U2C = self.U2 / (self.ct * self.l)
            U3C = self.U3 / (self.ct * self.l)
            U4C = self.U4 /  self.cq

            UC_vector = np.zeros((4, 1))
            UC_vector[0, 0] = U1C
            UC_vector[1, 0] = U2C
            UC_vector[2, 0] = U3C
            UC_vector[3, 0] = U4C

            omega_Matrix = np.zeros((4, 4))
            omega_Matrix[0, 0] = 1
            omega_Matrix[0, 1] = 1
            omega_Matrix[0, 2] = 1
            omega_Matrix[0, 3] = 1
            omega_Matrix[1, 1] = 1
            omega_Matrix[1, 3] = -1
            omega_Matrix[2, 0] = -1
            omega_Matrix[2, 2] = 1
            omega_Matrix[3, 0] = -1
            omega_Matrix[3, 1] = 1
            omega_Matrix[3, 2] = -1
            omega_Matrix[3, 3] = 1

            omega_Matrix_inverse = np.linalg.inv(omega_Matrix)
            omegas_vector = np.matmul(omega_Matrix_inverse, UC_vector)

            omega1P2 = omegas_vector[0, 0]
            omega2P2 = omegas_vector[1, 0]
            omega3P2 = omegas_vector[2, 0]
            omega4P2 = omegas_vector[3, 0]

            if omega1P2 <= 0 or omega2P2 <= 0 or omega3P2 <= 0 or omega4P2 <= 0:
                self.logger.info("You can't take a square root of a negative number")
                # print(
                #     "The problem might be that the trajectory is too chaotic or it might have large discontinuous jumps"
                # )
                # print("Try to make a smoother trajectory without discontinuous jumps")
                # print(
                #     "Other possible causes might be values for variables such as Ts, hz, innerDyn_length, px, py, pz"
                # )
                # print(
                #     "If problems occur, please download the files again, use the default settings and try to change values one by one."
                # )
                # exit()
                continue
            else:
                self.omega_values[0] = np.sqrt(omega1P2)
                self.omega_values[1] = np.sqrt(omega2P2)
                self.omega_values[2] = -np.sqrt(omega3P2)
                self.omega_values[3] = -np.sqrt(omega4P2)
                # Publish Commands
                self.publish_commands()

            # Compute the new total omega
            self.omega_total = sum(self.omega_values)
            self.controller_update_rate.sleep()
        
        
    def publish_commands(self):
        if self.throttle:
            self.rotor1_pub.publish(Float64(data=float(self.omega_values[0] )))
            self.rotor2_pub.publish(Float64(data=float(self.omega_values[1] )))
            self.rotor3_pub.publish(Float64(data=float(self.omega_values[2] )))
            self.rotor4_pub.publish(Float64(data=float(self.omega_values[3] )))
            self.logger.info('---\n\nheader')
            self.logger.info('Current state: XYZ: (%f, %f, %f)' % (self.x, self.y, self.z))
            self.logger.info('Current state: RPY: (%f, %f, %f)' % (self.roll, self.pitch, self.yaw))
            self.logger.info('Published command values: ' + ', '.join(map(str, self.omega_values)))
            logging.info(f'{self.x}, {self.y}, {self.z}, {self.roll}, {self.pitch}, {self.yaw}, {self.omega_values[0]}, {self.omega_values[1]}, {self.omega_values[2]}, {self.omega_values[3]}')
        else:
            self.logger.info('Throttle is unset !!')
        return

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
        
        self.states = np.array([self.vx, self.vy, self.vz, self.wx, self.wy, self.wz, self.x, self.y, self.z, self.roll, self.pitch, self.yaw])
        self.logger.info('self.z: %f' % self.states[8])
        

    def update_rpy(self, orientation_q):
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion(orientation_q)

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
            
        
    def move_to_height(self, height):
        self.desired_posn[2] = height
        self.logger.info('Desrired Posn set to: ' + ', '.join(map(str, self.desired_posn)))
        while(True):
            try:
                self.go_to_goal(self.desired_posn[0], self.desired_vel[0], self.desired_acc[0],
                                self.desired_posn[1], self.desired_vel[1], self.desired_acc[1],
                                self.desired_posn[2], self.desired_vel[2], self.desired_acc[2], self.desired_psi)
                self.controller_update_rate.sleep()
            except KeyboardInterrupt as E:
                self.logger.info('Error in move_to_height: %s' % E)
                break
        
        
    def follow_trajectory(self):
        for i in range(self.state_trajectory.shape[1]):
            state_vector = [self.state_trajectory[j][i] for j in range(self.state_trajectory.shape[0])]
            self.logger.info('Moving to: ' + ', '.join(map(str, state_vector)))
            self.go_to_goal(state_vector[0], state_vector[1], state_vector[2], state_vector[3], 
                            state_vector[4], state_vector[5], state_vector[6], state_vector[7], 
                            state_vector[8], state_vector[9])
            self.controller_update_rate.sleep()
            

        
        
def main(args=None):
    parser = argparse.ArgumentParser(description='MPC Node')
    parser.add_argument('--takeoff', type=float, default=0.0, help='takeoff height')
    parsed_args = parser.parse_args(args)
    
    rclpy.init(args=args)
    
    drone_node = DroneController()
    drone_node.start_odometry()
    drone_node.init_mpc()
    
    drone_node.move_to_height(parsed_args.takeoff)
        
    try:
        rclpy.spin(drone_node)
    except KeyboardInterrupt:
        drone_node.logger.info('Keyboard Interrupt')
    finally:
        drone_node.destroy_node()

if __name__ == '__main__':
    main()
