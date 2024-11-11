#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
from casadi import *
import time, math
from transforms3d.euler import quat2euler

class DroneMPCController(Node):
    def __init__(self):
        super().__init__('drone_mpc_controller')
        
        # System parameters
        self.m_quad = 1.5  # mass of quadrotor
        self.m_load = 0.1  # mass of payload
        self.g = 9.81      # gravity
        self.L = 1.0       # cable length
        
        # MPC parameters
        self.N = 10        # prediction horizon
        self.dt = 0.1      # time step
        
        # State and control constraints
        self.x_min = -10.0
        self.x_max = 10.0
        self.y_min = -10.0
        self.y_max = 10.0
        self.z_min = 0.0
        self.z_max = 10.0
        self.thrust_min = self.m_quad * self.g * 0.5  # Minimum thrust
        self.thrust_max = self.m_quad * self.g * 1.5  # Maximum thrust
        self.angle_min = -np.pi/6  # More conservative angle limits
        self.angle_max = np.pi/6
        
        self.motor_thrust_constant = 0.000018937
        
        # Initialize state vectors
        self.quad_state = np.zeros(12)  # [x,y,z,vx,vy,vz,roll,pitch,yaw,wx,wy,wz]
        self.load_state = np.zeros(6)   # [x,y,z,vx,vy,vz]
        
        # Publishers
        self.rotor1_pub = self.create_publisher(Float64, '/drone_v1/rotor_0_joint/cmd_roll', 10)
        self.rotor2_pub = self.create_publisher(Float64, '/drone_v1/rotor_1_joint/cmd_roll', 10)
        self.rotor3_pub = self.create_publisher(Float64, '/drone_v1/rotor_2_joint/cmd_roll', 10)
        self.rotor4_pub = self.create_publisher(Float64, '/drone_v1/rotor_3_joint/cmd_roll', 10)
        
        # Subscribers
        self.drone_odom_sub = self.create_subscription(Odometry, '/drone_v1/odometry', self.drone_odom_callback, 10)
        self.load_odom_sub = self.create_subscription(Odometry, '/cube/odometry', self.load_odom_callback, 10)
        
        # Cost matrices
        # Q: State error weights (18x18 matrix)
        self.Q = np.diag([
            10.0, 10.0, 20.0,      # position error weights (x,y,z)
            1.0, 1.0, 1.0,         # velocity error weights
            5.0, 5.0, 5.0,         # attitude error weights
            0.1, 0.1, 0.1,         # angular velocity error weights
            10.0, 10.0, 20.0,      # load position error weights
            1.0, 1.0, 1.0          # load velocity error weights
        ])

        # R: Control input weights (4x4 matrix)
        self.R = np.diag([
            0.01,   # thrust weight
            1.0,    # roll command weight
            1.0,    # pitch command weight
            1.0     # yaw command weight
        ])
        
        # Setup MPC solver
        self.setup_mpc()
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def setup_mpc(self):
        # State variables
        x = SX.sym('x', 18)  # Combined state [quad_state(12); load_state(6)]
        u = SX.sym('u', 4)   # Control inputs [thrust, roll, pitch, yaw]
        
        # System dynamics
        x_dot = self.system_dynamics(x, u)
        
        # Formulate discrete time dynamics
        f = Function('f', [x, u], [x_dot])
        
        # Decision variables
        self.opti = Opti()
        self.X = self.opti.variable(18, self.N+1)  # state trajectory
        self.U = self.opti.variable(4, self.N)     # control trajectory
        
        # Parameters for the target state
        self.P = self.opti.parameter(18, 1)
        
        # Convert numpy arrays to CasADi matrices
        Q = DM(self.Q)
        R = DM(self.R)
        
        # Objective function
        obj = 0
        for k in range(self.N):
            state_error = self.X[:,k] - self.P
            control_effort = self.U[:,k]
            obj += state_error.T @ Q @ state_error + control_effort.T @ R @ control_effort
            
        self.opti.minimize(obj)
        
        # Dynamic constraints with simpler integration scheme
        for k in range(self.N):
            # Forward Euler integration (simpler than RK4 for initial testing)
            x_next = self.X[:,k] + self.dt * self.system_dynamics(self.X[:,k], self.U[:,k])
            self.opti.subject_to(self.X[:,k+1] == x_next)
        
        # State and control constraints
        for k in range(self.N+1):
            self.opti.subject_to(self.X[0:3,k] >= [self.x_min, self.y_min, self.z_min])
            self.opti.subject_to(self.X[0:3,k] <= [self.x_max, self.y_max, self.z_max])
            
        for k in range(self.N):
            self.opti.subject_to(self.U[0,k] >= self.thrust_min)
            self.opti.subject_to(self.U[0,k] <= self.thrust_max)
            self.opti.subject_to(self.U[1:4,k] >= self.angle_min)
            self.opti.subject_to(self.U[1:4,k] <= self.angle_max)
            
        # Initialize with a reasonable guess
        for k in range(self.N+1):
            self.opti.set_initial(self.X[:,k], np.zeros(18))
        for k in range(self.N):
            self.opti.set_initial(self.U[:,k], [self.m_quad * self.g, 0, 0, 0])  # Hover thrust and zero angles

        # Add initial state constraint
        self.x0 = self.opti.parameter(18, 1)
        self.opti.subject_to(self.X[:,0] == self.x0)

        # Solver options with more conservative settings
        opts = {
            'ipopt.print_level': 5,  # Increase for more debug info
            'ipopt.max_iter': 200,
            'ipopt.tol': 1e-3,       # Looser tolerance
            'ipopt.acceptable_tol': 1e-2,
            'ipopt.acceptable_iter': 3,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.mu_strategy': 'monotone',
            'ipopt.alpha_for_y': 'safer-min-dual-infeas',
            'ipopt.bound_mult_init_method': 'mu-based',
            'ipopt.bound_push': 0.01,
            'ipopt.bound_frac': 0.01
        }
        self.opti.solver('ipopt', opts)
        
    def system_dynamics(self, x, u):
        """
        Highly simplified system dynamics for initial testing
        """
        # Extract states
        quad_pos = x[0:3]
        quad_vel = x[3:6]
        quad_angles = x[6:9]
        quad_omega = x[9:12]
        load_pos = x[12:15]
        load_vel = x[15:18]
        
        # Extract controls (with bounds)
        thrust = fmin(self.thrust_max, fmax(self.thrust_min, u[0]))
        roll_cmd = fmin(self.angle_max, fmax(self.angle_min, u[1]))
        pitch_cmd = fmin(self.angle_max, fmax(self.angle_min, u[2]))
        yaw_cmd = fmin(self.angle_max, fmax(self.angle_min, u[3]))
        
        # Very simple quad dynamics
        quad_acc = vertcat(
            pitch_cmd,  # Simplified pitch control
            -roll_cmd,  # Simplified roll control
            (thrust/self.m_quad) - self.g  # Vertical dynamics
        )
        
        # Simple load dynamics (pendulum-like)
        load_acc = vertcat(
            quad_acc[0],  # Load follows quad horizontally
            quad_acc[1],
            -self.g      # Vertical motion only affected by gravity
        )
        
        # Simple angular dynamics
        angle_rates = quad_omega
        omega_dot = -0.5 * quad_omega  # Simple damping
        
        return vertcat(
            quad_vel,
            quad_acc,
            angle_rates,
            omega_dot,
            load_vel,
            load_acc
        )
        
    def drone_odom_callback(self, msg):
        # Update quadrotor state from odometry message
        self.quad_state[0:3] = [msg.pose.pose.position.x,
                               msg.pose.pose.position.y,
                               msg.pose.pose.position.z]
                               
        # Convert quaternion to euler angles
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]
        euler = quat2euler(quat, 'sxyz')  # 'sxyz' specifies the rotation sequence
        
        self.quad_state[6:9] = euler
        
        self.quad_state[3:6] = [msg.twist.twist.linear.x,
                               msg.twist.twist.linear.y,
                               msg.twist.twist.linear.z]
                               
        self.quad_state[9:12] = [msg.twist.twist.angular.x,
                                msg.twist.twist.angular.y,
                                msg.twist.twist.angular.z]
                                
    def load_odom_callback(self, msg):
        # Update payload state from odometry message
        self.load_state[0:3] = [msg.pose.pose.position.x,
                               msg.pose.pose.position.y,
                               msg.pose.pose.position.z]
                               
        self.load_state[3:6] = [msg.twist.twist.linear.x,
                               msg.twist.twist.linear.y,
                               msg.twist.twist.linear.z]
                               
    def control_loop(self):
        try:
            current_state = np.concatenate([self.quad_state, self.load_state])
            
            # Set parameter values
            self.opti.set_value(self.x0, current_state)
            
            # Set target state (more conservative)
            target_state = np.zeros(18)
            target_state[2] = 1.0  # Only target height initially
            self.opti.set_value(self.P, target_state)
            
            # Warm start from previous solution if available
            hover_thrust = self.m_quad * self.g
            self.opti.set_initial(self.X[:,0], current_state)
            for k in range(self.N):
                self.opti.set_initial(self.U[:,k], [hover_thrust, 0, 0, 0])
            
            # Solve with timeout
            sol = self.opti.solve()
            
            # Extract and apply first control input
            u_optimal = sol.value(self.U[:,0])
            rotor_commands = self.mpc_to_rotor_commands(u_optimal)
            self.publish_commands(rotor_commands)
            
        except Exception as e:
            self.get_logger().error(f'MPC solve failed: {str(e)}')
            # Safe fallback control
            hover_commands = self.mpc_to_rotor_commands([self.m_quad * self.g, 0, 0, 0])
            self.publish_commands(hover_commands)
            
    def mpc_to_rotor_commands(self, u):
        """
        Convert MPC control inputs to individual rotor commands
        """
        thrust = math.sqrt(u[0] / self.motor_thrust_constant)
        roll = u[1]
        pitch = u[2]
        yaw = u[3]
        
        # Simple mixing matrix (modify based on your drone configuration)
        mixing_matrix = np.array([
            [ 1,  1,  1,  1],  # thrust
            [ 1, -1, -1,  1],  # roll
            [ 1,  1, -1, -1],  # pitch
            [ 1, -1,  1, -1]   # yaw
        ])
        
        commands = mixing_matrix @ np.array([thrust, roll, pitch, yaw])
        return np.clip(commands, 0, 1000)  # adjust limits as needed
        
    def publish_commands(self, commands):
        self.rotor1_pub.publish(Float64(data=float(commands[0])))
        self.rotor2_pub.publish(Float64(data=float(commands[1])))
        self.rotor3_pub.publish(Float64(data=float(commands[2])))
        self.rotor4_pub.publish(Float64(data=float(commands[3])))

def main(args=None):
    rclpy.init(args=args)
    controller = DroneMPCController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
