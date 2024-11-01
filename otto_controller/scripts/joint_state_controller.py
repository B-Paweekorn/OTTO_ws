#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class Controller(Node):
    def __init__(self):
        super().__init__('setpoint_to_effort_node')
        self.create_subscription(Float64MultiArray,'/setpoint',self.setpoint_callback,10)
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)

        self.effort_publisher = self.create_publisher(Float64MultiArray,'/effort_controller/commands',10)

        # HL, KL, VHL, WL, HR, KR, VHR, WR
        self.setpoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state = []


    def feedback_callback(self, msg):
        joint_names = msg.name
        feedback_positions = msg.position
        feedback_velocities = msg.velocity
        feedback_efforts = msg.effort
        
        self.joint_states = {
            'names': joint_names,
            'positions': feedback_positions,
            'velocities': feedback_velocities,
            'efforts': feedback_efforts
        }
                
    def setpoint_callback(self, msg):
        self.setpoint = msg.data
        
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
