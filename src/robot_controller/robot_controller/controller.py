import matplotlib
import matplotlib.pyplot as plt
import math 
import rclpy
from rclpy.node import Node
import numpy as np

# These message types are required
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker

class kinematics:        
    def rot_x(theta):
        rad = math.radians(theta)
        c = np.cos(rad)
        s = np.sin(rad)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])
        
    def rot_y(theta):
        rad = math.radians(theta)
        c = np.cos(rad)
        s = np.sin(rad)
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1]
        ])
    
    def rot_z(theta):
        rad = math.radians(theta)
        c = np.cos(rad)
        s = np.sin(rad)
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
    def trans(x,y,z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

# PID Math
class PID_controller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.last_error = 0
        self.integral_sum = 0
    
    
    def update(self, setpoint, measured_value, delta_t):
        # Calculate Error
        self.error= setpoint - measured_value
        
        # Find P term
        P_term = self.Kp * self.error
        
        # Find I term
        self.integral_sum +=self.error* delta_t
        I_term = self.Ki * self.integral_sum
        
        # Find D term
        d_error = (self.error - self.last_error) / delta_t
        D_term = self.Kd * d_error
        
        # Find output
        output = P_term + I_term + D_term
        
        self.last_error = self.error
        
        return output
    
# Point Math
# class point_calculations(self):
#     def calculations(self):
#         self.elbow.x = self.l1 * math.cos(self.current_position.position[2])
#         self.elbow.y = self.l1 * math.sin(self.current_position.position[3])
#         self.
    
class RobotController(Node):
    def __init__(self):
        super().__init__('controller')
        
        q1 = math.radians(0)
        q2 = math.radians(45)
        q3 = math.radians(90)
        L1 = .3
        L2 = .3
        L3 = .2
        
        # T_01 = kinematics.rot_z(q1) @ kinematics.trans(0,0,0.3)
        # T_12 = kinematics.rot_y(q2) @ kinematics.trans(0.3,0,0)
        # T_global = T_01 @ T_12
        
        # position = T_global[:3,3]
        # print(f"tip is at {position}")
        
        # Base -> Joint 1 (Rotation Z) -> Link 1 Tip (Translation Z)
        T_01 = kinematics.rot_z(q1) @ kinematics.trans(0, 0, L1)

        # Link 1 -> Joint 2 (Rotation Y) -> Link 2 Tip (Translation X)
        T_12 = kinematics.rot_y(q2) @ kinematics.trans(L2, 0, 0)

        # Link 2 -> Joint 3 (Rotation Y) -> End Effector (Translation X)
        T_23 = kinematics.rot_y(q3) @ kinematics.trans(L3, 0, 0)
        
        
        # Position of Elbow (End of Link 1) 
        pos_elbow_matrix = T_01
        xyz_elbow = pos_elbow_matrix[:3, 3]

        # Position of Wrist (End of Link 2)
        pos_wrist_matrix = T_01 @ T_12
        xyz_wrist = pos_wrist_matrix[:3, 3]

        # Position of Tip (End Effector)
        pos_tip_matrix = T_01 @ T_12 @ T_23
        xyz_tip = pos_tip_matrix[:3, 3]
        
        print(f'elbow: {xyz_elbow[0]}')
        print(f'wrist: {xyz_wrist}')
        print(f'tip: {xyz_tip}')
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)        
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        
        msg.name = ['joint1','joint2','joint3']
        
        msg.position = [float(q1), float(q2), float(q3)]
        
        self.joint_pub.publish(msg)
        
        
        
    # def arm_publisher(self):
        # marker = Marker()
        # marker.header.frame_id = "world"
        # marker.header.stamp = self.get_clock().now().to_msg()
        
        # marker.id = 0 # This gives the marker an ID
        # marker.type = Marker.LINE_STRIP
        # marker.action = Marker.ADD
        
        # # Set line width
        # marker.scale.x = 0.05
        
        # # Set line color (Blue)
        # marker.color.b = 1.0
        # marker.color.r = 0.5
        # marker.color.a = 1.0
        
        # p_origin = Point() # Defines the origin of link 1
        # p_origin.x = xyz_elbow[0]
        # p_origin.y = xyz_elbow[1]
        # p_origin.z = xyz_elbow[2]
        
        # # Define position of the elbow (the point where l1 and l2 connect)
        # p_elbow = Point()
        # p_elbow.x = xyz_wrist[0]
        # p_elbow.y = xyz_wrist[1]
        # p_elbow.z = xyz_wrist[2]
        
        
        # p_hand = Point()
        # p_hand.x = xyz_tip[0]
        # p_hand.y = xyz_tip[1]
        # p_hand.z = xyz_tip[2]
        
        # # Add the points to the marker's list    
        # marker.points = [p_origin, p_elbow, p_hand] 
        
        # self.marker_publisher_.publish(marker)
        
        
        
def main(args=None):
    rclpy.init(args=args)
    
    robot_controller = RobotController()
    
    # make it so the node loops until we exit it
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    
    robot_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()