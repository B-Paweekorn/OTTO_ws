#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
import signal
import tf_transformations
import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates, ModelStates

# ROBOT PARAMETER
class Controller(Node):
    def __init__(self):
        super().__init__('joint_level_control')
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.dt = 1/1000

        self.joint_state = {}

        # KL, VHL, WL, KR, VHR, WR
        self.dof = 2
        self.q_d = [0.0] * self.dof
        self.ctrl_input = [0.0] * self.dof

        # imu
        self.imu_angle = None
        self.gyro = [0.0, 0.0, 0.0] # rpy
        self.accel = [0.0, 0.0, 0.0]

        self.dth_s_filtered = 0.0
        self.alpha = 0.1
        # command
        self.vx_cmd = 0
        self.prev_vx = 0
        self.w_cmd = 0
        self.roll_cmd_i = 0.0
        self.isCFAL = 0.0
        
        # x_s
        self.x_s = 0
        self.x_cmd = 0
        self.x_i = 0

        self.whl_Rz = 0.0
        self.whl_Lz = 0.0

        self.th_si = 0.0

        # yaw
        self.yaw_cmd = 0
        self.prev_yaw = 0.0
        self.yaw_offset = 0.0

        self.roll_s_i = 0.0
        # LEG variables -----------------------------------------
        self.targL = [0.0, -0.28]
        self.targR = [0.0, -0.28]
        self.prev_targ = 0.0
        self.q_kneeL_des = 0.0
        self.q_kneeR_des = 0.0
        self.q_hipL_des = 0.0
        self.q_hipR_des = 0.0
        self.singularity = False

        # Robot parameters -----------------------------------------
        self.r = 0.086
        self.d = 0.270
        self.l = 0.2828
        self.g = 9.81
        self.m_b = 1.5
        self.mu = 0.001

        self.l1 = 0.2
        self.l2 = 0.2

        # Gain parameters -------------- # state: x th1 yaw dx dth1 dyaw x_i yaw_i
        
        self.K_lqr = np.array([
        [-6.0, -20.63, -2.0, -5.019, -4.37, -0.7, -1.707, 0],
        [-6.0, -20.63, 2.0, -5.019, -4.37, 0.7, -1.707, 0]
        ])

        self.K_height = 7

        self.prev_th_s = 0.0
        # ROS2 -----------------------------------------

        self.timer = self.create_timer(self.dt, self.timer_callback)

        # ROS2 Wheel
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)
        self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.create_subscription(LinkStates, '/gazebo/link_states', self.link_states_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray,'/effort_controller/commands',10)
        self.effort_msg = Float64MultiArray()
    
        # ROS2 Leg
        self.pos_cmd_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.create_subscription(Float64MultiArray, '/legpose', self.legpose_callback, 10)

        self.pos_cmd_msg = Float64MultiArray()

        # Ground Truth
        self.create_subscription(LinkStates, '/gazebo/link_states', self.link_states_callback, 10)

    def timer_callback(self):
        # state: x th1 th2 yaw dx dth1 dth2 dyaw x_i yaw_i
        # u: l_wheel r_wheel
        try:
            # =============== State for Standing =================
            vx_s = (self.joint_state["qd"][2] + self.joint_state["qd"][5])*0.5*self.r
            dth_s = self.gyro[1]
            self.dth_s_filtered = self.alpha * self.gyro[1] + (1 - self.alpha) * self.dth_s_filtered  # Low-pass filter
            w_s = self.gyro[2] # 0.5*(self.joint_state["qd"][5] - self.joint_state["qd"][2])*self.r/self.d 

            self.x_s += vx_s * self.dt

            roll_s = self.imu_angle[0]
            th_s = self.imu_angle[1]
            yaw_s = self.imu_angle[2]

            # =============== Running State =================
            if self.get_clock().now().seconds_nanoseconds()[0] - self.start_time >= 10:
                th_s =  0.5 * (self.forward_kinematics([self.joint_state["q"][0] + self.imu_angle[1], self.joint_state["q"][1]])[1] + self.forward_kinematics([self.joint_state["q"][3] + self.imu_angle[1], self.joint_state["q"][4]])[1])
                dth_s = (th_s - self.prev_th_s)/self.dt
                self.prev_th_s = th_s

            # =============== Command =================
            self.x_cmd += self.vx_cmd * self.dt
            self.yaw_cmd += self.w_cmd * self.dt
            th_cmd = np.arcsin((2*self.mu*self.vx_cmd/self.r)/(self.m_b*self.l*self.g))*0
            ffw = (self.m_b*self.l*self.g*np.sin(th_cmd) - 2*self.mu*self.vx_cmd/self.r)/2

            # =============== LQR Controller =================
            """ x th yaw
                vx dth dyaw
                x_i yaw_i """
            
            self.x_i += (self.x_s - self.x_cmd) * self.dt

            state_d = np.array([self.x_cmd, th_cmd, self.yaw_cmd, 
                                self.vx_cmd, 0, self.w_cmd,
                                0, 0])

            state = np.array([self.x_s, th_s, yaw_s,
                              vx_s, dth_s, w_s,
                              self.x_i, 0])

            error = state_d - state
            U = self.K_lqr @ error

            self.ctrl_input[0] = ffw + U[0]
            self.ctrl_input[1] = ffw + U[1]

            wheel_lim = 3.0
            self.ctrl_input = np.clip(self.ctrl_input, -wheel_lim, wheel_lim)

            self.effort_msg.data = self.ctrl_input.tolist()
            self.effort_publisher.publish(self.effort_msg)

            # ======================= LEAN ANGLE TASK =======================
            
            if self.get_clock().now().seconds_nanoseconds()[0] - self.start_time >= 10:
                legL_s = np.array([self.q_kneeL_des, self.q_hipL_des])
                legR_s = np.array([self.q_kneeR_des, self.q_hipR_des])
                ROLL_KP = 5.0
                ROLL_KI = 4.0 * 0
                ROLL_KD = 0.5
                ROLL_SATURATION = 45 # DEG
                TORSO_KI = 0.05

                if self.isCFAL:
                    roll_cmd = np.arctan(self.vx_cmd*self.w_cmd/self.g)
                else:
                    roll_cmd = 0.0

                error = roll_cmd - self.roll_cmd_i
                
                if abs(roll_cmd) >= np.deg2rad(ROLL_SATURATION):
                    roll_cmd = np.deg2rad(ROLL_SATURATION) * np.sign(roll_cmd)

                if abs(error) > 0.01:
                    if self.roll_cmd_i < roll_cmd:
                        self.roll_cmd_i = min(self.roll_cmd_i + 0.001, roll_cmd)
                    elif self.roll_cmd_i > roll_cmd:
                        self.roll_cmd_i = max(self.roll_cmd_i - 0.001, roll_cmd)

                roll_s_p = roll_s * ROLL_KP

                if not self.isCFAL:
                    self.roll_s_i += roll_s * self.dt * ROLL_KI
                else:
                    self.roll_s_i = 0
                    
                roll_s_d = self.gyro[0] * ROLL_KD

                roll_u = roll_s_p + roll_s_d + self.roll_s_i
                
                if abs(roll_u) >= np.deg2rad(ROLL_SATURATION):
                    roll_u = np.deg2rad(ROLL_SATURATION) * np.sign(roll_u)
                


                Rz = 0.5*self.d*np.tan(roll_u * (1 - self.isCFAL) + self.roll_cmd_i) + self.l
                Lz = 2*self.l - Rz
                
                self.targR[1] = -Rz
                self.targL[1] = -Lz

                self.th_si += self.imu_angle[1] * TORSO_KI 

                qd_L = self.leg_controller(legL_s, np.array(self.targL))
                self.q_kneeL_des += (qd_L[0] + self.th_si*0) * self.dt
                self.q_hipL_des += qd_L[1] * self.dt
                
                qd_R = self.leg_controller(legR_s, np.array(self.targR))
                self.q_kneeR_des += (qd_R[0] + self.th_si*0) * self.dt
                self.q_hipR_des += qd_R[1] * self.dt

                if not self.singularity:
                    self.pos_cmd_msg.data = [self.q_kneeL_des + self.th_si, self.q_hipL_des, self.q_kneeR_des + self.th_si, self.q_hipR_des]
                    self.pos_cmd_pub.publish(self.pos_cmd_msg)
            
        except (TypeError, IndexError, KeyError) as e:
            self.get_logger().error(f"Error updating control input: {e}")

    def forward_kinematics(self, q):
        x = -self.l1*np.cos(np.deg2rad(45) - q[0]) + self.l2*np.cos(np.deg2rad(45) + q[1] + q[0])
        z = -self.l1*np.sin(np.deg2rad(45) - q[0]) - self.l2*np.sin(np.deg2rad(45) + q[1] + q[0])
        theta = np.arctan(x/z)
        return np.array([x,z]), theta
    
    def fnc_jacobian(self, q):
        J = np.array([[-self.l1*np.sin(np.deg2rad(45)-q[0]) - self.l2*np.sin(np.deg2rad(45)+q[0]+q[1]), -self.l2*np.sin(np.deg2rad(45)+q[0]+q[1])], 
                      [self.l1*np.cos(np.deg2rad(45)-q[0]) - self.l2*np.cos(np.deg2rad(45)+q[0]+q[1]), -self.l2*np.cos(np.deg2rad(45)+q[0]+q[1])]])

        J_inv = np.linalg.inv(J)
        return J, J_inv
    
    def link_states_callback(self, msg):
        try:
            index_L = msg.name.index("otto::wheel_linkL")
            self.whl_Lz = msg.pose[index_L].position.z
            index_R = msg.name.index("otto::wheel_linkR")
            self.whl_Rz = msg.pose[index_R].position.z

        except ValueError as e:
            self.get_logger().warn(f"Wheel link not found in /gazebo/link_states: {e}")

    def leg_controller(self, curr, targ):
        if abs(np.linalg.det(self.fnc_jacobian(curr)[0])) < 0.0001:
            print("Singularlity!!")
            self.singularity = True
            return np.zeros((2,1))
        err = targ - self.forward_kinematics(curr)[0] # 1 x 2
        v = err * self.K_height

        q_dot = np.matmul(self.fnc_jacobian(curr)[1], np.transpose(v))
        return q_dot # 2 x 1

    def legpose_callback(self, msg):
        self.l = -msg.data[1]
        if msg.data[3] > -0.25:
            self.isCFAL = 1.0
            self.roll_s_i = 0
        else:
            if abs(self.roll_cmd_i) < 0.05:
                self.isCFAL = 0.0

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
        # self.imu_angle = [orientation_q.roll, orientation_q.pitch, orientation_q.yaw]
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
        r, p, y = self.imu_angle
        self.imu_angle[2] = self.unwrap_yaw(y)

        rotation_matrix = np.array([
            [np.cos(p), 0, -np.cos(r)*np.sin(p)],
            [0, 1, np.sin(r)],
            [np.sin(p), 0, np.cos(r)*np.cos(p)]
        ])

        # rotation_matrix = np.array([
        #     [1, 0, -np.sin(p)],
        #     [0, np.cos(r), np.cos(p)*np.cos(r)],
        #     [0, -np.sin(r), np.cos(r)*np.cos(p)]
        # ])

        gyro_sensor = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        self.gyro = np.matmul(rotation_matrix.T, gyro_sensor)

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
