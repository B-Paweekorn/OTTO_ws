#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
import signal
import tf_transformations
import numpy as np
from geometry_msgs.msg import Twist

# ROBOT PARAMETER
class Controller(Node):
    def __init__(self):
        super().__init__('joint_level_control')
        self.dt = 1/2000

        self.joint_state = {}

        # KL, VHL, WL, KR, VHR, WR
        self.dof = 2
        self.q_d = [0.0] * self.dof
        self.ctrl_input = [0.0] * self.dof

        # imu
        self.imu_angle = None
        self.gyro = [0.0, 0.0, 0.0] # rpy
        self.accel = [0.0, 0.0, 0.0]
        
        # command
        self.vx_cmd = 0
        self.prev_vx = 0
        self.w_cmd = 0

        # x_s
        self.x_s = 0
        self.x_cmd = 0
        self.x_i = 0

        self.yaw_cmd = 0
        self.prev_yaw = 0.0
        self.yaw_offset = 0.0
        # robot geometry
        self.r = 0.086
        self.d = 0.2254
        self.l = 0.2828
        self.g = 9.81
        self.m_b = 1.5
        self.mu = 0.01
        
        # state: x th1 yaw dx dth1 dyaw x_i yaw_i
        self.K_lqr = np.array([
        [-6.0, -30.63, -6.1623, -8.019, -10.5305, -1.1979, 0.707, 0],
        [-6.0, -30.63, 6.1623, -8.019, -10.5305, 1.1979, 0.707, 0]
        ])

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)
        self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray,'/effort_controller/commands',10)

        self.effort_msg = Float64MultiArray()
    
    def timer_callback(self):
        # state: x th1 th2 yaw dx dth1 dth2 dyaw x_i yaw_i
        # u: l_wheel r_wheel
        try:
            # =============== State =================
            vx_s = (self.joint_state["qd"][2] + self.joint_state["qd"][5])*0.5*self.r
            dth_s = self.gyro[1]
            w_s = self.gyro[2]

            self.x_s += vx_s * self.dt
            th_s = self.imu_angle[1]
            yaw_s = self.imu_angle[2]
            # self.x_i += (self.x_cmd - self.x_s)*self.dt

            # if abs(self.vx_cmd) <= 0.01 and self.prev_vx != 0.0:
            #     self.xdes_s = self.x_s + 0.5*vx_s
            #     self.x_i = 0
            # self.prev_vx = self.vx_cmd

            # =============== Command =================
            self.x_cmd += self.vx_cmd * self.dt
            self.yaw_cmd += self.w_cmd * self.dt
            th_cmd = np.arcsin((2*self.mu*self.vx_cmd/self.r)/(self.m_b*self.l*self.g))
            ffw = (self.m_b*self.l*self.g*np.sin(th_cmd) - 2*self.mu*self.vx_cmd/self.r)/2

            # =============== LQR Controller =================
            """ x th yaw
                vx dth dyaw
                x_i yaw_i """
            
            state_d = np.array([self.x_cmd, th_cmd, self.yaw_cmd, 
                                0, 0, 0,
                                0, 0])
            
            state = np.array([self.x_s, th_s, yaw_s,
                             vx_s, dth_s, w_s,
                             self.x_i, 0])

            error = state_d - state
            U = self.K_lqr @ error

            self.ctrl_input[0] = ffw + U[0]
            self.ctrl_input[1] = ffw + U[1]

            wheel_lim = 2.2

            if self.ctrl_input[0] > wheel_lim:
                self.ctrl_input[0] = wheel_lim
            elif self.ctrl_input[0] < -wheel_lim:
                self.ctrl_input[0] = -wheel_lim

            if self.ctrl_input[1] > wheel_lim:
                self.ctrl_input[1] = wheel_lim
            elif self.ctrl_input[1] < -wheel_lim:
                self.ctrl_input[1] = -wheel_lim

            self.effort_msg.data = self.ctrl_input
            self.effort_publisher.publish(self.effort_msg)

        except (TypeError, IndexError, KeyError) as e:
            self.get_logger().error(f"Error updating control input: {e}")

    def cmd_vel_callback(self, msg):
        self.vx_cmd = msg.linear.x
        self.w_cmd = msg.angular.z

    def feedback_callback(self, msg):
        joint_names = msg.name
        joint_q = msg.position
        joint_qd = msg.velocity
        joint_eff = msg.effort

        order = [
            'knee_jointL', 'virtualhip_jointL', 'wheel_jointL',
            'knee_jointR', 'virtualhip_jointR', 'wheel_jointR'
        ]

        joint_data = {
            name: (q, qd, effort)
            for name, q, qd, effort in zip(joint_names, joint_q, joint_qd, joint_eff)
        }

        self.joint_state = {
            'names': order,
            'q': [joint_data[joint][0] for joint in order],
            'qd': [joint_data[joint][1] for joint in order],
            'efforts': [joint_data[joint][2] for joint in order]
        }
                
    def imu_callback(self, msg):
        orientation_q = msg.orientation
        quaternion = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]
        
        self.imu_angle = list(tf_transformations.euler_from_quaternion(quaternion))
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
        r, p, y = self.imu_angle
        self.imu_angle[2] = self.unwrap_yaw(y)

        rotation_matrix = np.array([
            [np.cos(p), 0, -np.cos(r)*np.sin(p)],
            [0, 1, np.sin(r)],
            [np.sin(p), 0, np.cos(r)*np.cos(p)]
        ])
        gyro_sensor = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        self.gyro = np.dot(rotation_matrix.T, gyro_sensor)

        self.accel[0] = msg.linear_acceleration.x
        self.accel[1] = msg.linear_acceleration.y
        self.accel[2] = msg.linear_acceleration.z
    
    def unwrap_yaw(self, yaw):
        delta_yaw = yaw - self.prev_yaw
        if delta_yaw > np.pi:
            self.yaw_offset -= 2 * np.pi
        elif delta_yaw < -np.pi:
            self.yaw_offset += 2 * np.pi
        
        self.prev_yaw = yaw
        return yaw + self.yaw_offset
    
    def stop_robot(self):
        self.ctrl_input = [0.0] * self.dof
        zero_effort_msg = Float64MultiArray()
        zero_effort_msg.data = self.ctrl_input
        self.effort_publisher.publish(zero_effort_msg)
        print("Robot stopped with zero effort.")

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    def signal_handler(sig, frame):
        print("Caught SIGINT, stopping the robot...")
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
