import matplotlib
import matplotlib.pyplot as plt
import math 
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker
from robot_controller.planner import RRTplanner

class kinematics:        
    def rot_x(theta):
        rad = theta
        c = np.cos(rad)
        s = np.sin(rad)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])
        
    def rot_y(theta):
        rad = theta
        c = np.cos(rad)
        s = np.sin(rad)
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1]
        ])
    
    def rot_z(theta):
        rad = theta
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
        self.timer = self.create_timer(1/90,self.control_loop)
        
        self.q1 = math.radians(0)
        self.q2 = math.radians(0)
        self.q3 = math.radians(0)
        
        self.L1 = .3
        self.L2 = .3
        self.L3 = .2        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        #################################################
        self.pid_1 = PID_controller(Kp=3,Ki=0,Kd=0.20)
        self.pid_2 = PID_controller(Kp=2,Ki=0,Kd=0.10)
        self.pid_3 = PID_controller(Kp=1,Ki=0,Kd=0.0)
        #################################################
        
        self.last_time = self.get_clock().now()
        self.vel_pub = self.create_publisher(Float64MultiArray, 'joint_velocities', 10)
        
        # Indexing/tolerance
        self.current_idx = 0
        self.tolerance = math.radians(1)
        self.required_time = 0.1 # How long the arm is required to be within tolerance
        self.stable_start_time = None
        self.move_start_time = self.get_clock().now()
        self.lap_time = 0
        
        # Current Angle
        self.current_JS = JointState()
        self.name = ['joint1','joint2','joint3']
        self.current_JS.position = [0.0,0.0,0.0]

        # # Define home angles
        # home = JointState()
        # home.name = ['shoulder_joint','elbow_joint']
        # home.position = [0.0,0.0,0.0]
        
        # #####################################################################
        # pick = JointState()
        # pick.name = ['joint1','joint2','joint3']
        # pick.position = [math.radians(0),math.radians(120),math.radians(60)]
            
        # place = JointState()
        # place.name = ['joint1','joint2','joint3']
        # place.position = [math.radians(135), math.radians(45), math.radians(90)]
        # #####################################################################
        
        start_conf = [0.0,0.0,0.0] # Home
        pick_conf = [math.radians(30), math.radians(90), math.radians(0)]

        # define obstacles
        obstacles = [[1.0, 0.0, 1.0, 0.1]]
        
        # Display Obstacle
        self.marker_pub = self.create_publisher(Marker, '/spheres', 10)
        self.publish_sphere()
        
        # Limits (from urdf)
        joint_limits = {
            'joint1': (-3.14, 3.14),
            'joint2': (-1.57, 1.57),
            'joint3': (-1.57, 1.57)
        }
        
        self.viz_pub = self.create_publisher(Marker, 'rrt_tree', 10)

        # initialize the planner
        self.get_logger().info('Starting RRT Planner...')
        rrt = RRTplanner(start_conf,pick_conf, obstacles, joint_limits, self.viz_pub)
        
        # run the planner
        path_list = rrt.plan()
        
        self.waypoints = []
        
        if path_list is None:
            self.get_logger().error("RRT failed to find a path :(")
        else:
            self.get_logger().info(f"Path found with {len(path_list)} steps!")
            
            for point in path_list:
                wp = JointState()
                wp.name = ['joint1', 'joint2', 'joint3']
                wp.position = point
                self.waypoints.append(wp)
        
        # Creates a list of the targets
        # self.waypoints = [home, pick, place]

        # Trajectory settings
        # self.move_duration = 3.0 # Take 3 seconds to move between points
        
        # Track where we started the current motion
        self.start_angles = [0.0, 0.0, 0.0]
        
        # Creates a subscriber node that listens to the joint_states topic, 
        # expecting a "JointState" message. Each time that it receives one, 
        # it calls the self.joint_callback function
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.sensor_callback,
            10
        )
        
    def publish_sphere(self):
        obs_marker = Marker()
        obs_marker.header.frame_id = "base_link"
        obs_marker.header.stamp = self.get_clock().now().to_msg()
        obs_marker.type = Marker.SPHERE
        obs_marker.action = Marker.ADD
        obs_marker.ns = "obstacles"
        obs_marker.id = 1

        obs_marker.pose.position.x = 1.0
        obs_marker.pose.position.y = 0.0
        obs_marker.pose.position.z = 1.0
        
        obs_marker.pose.orientation.x = 0.0
        obs_marker.pose.orientation.y = 0.0
        obs_marker.pose.orientation.z = 0.0
        obs_marker.pose.orientation.w = 1.0
        
        obs_marker.scale.x = .2
        obs_marker.scale.y = .2
        obs_marker.scale.z = .2
        
        obs_marker.color.a = 0.8
        obs_marker.color.r = 1.0
        obs_marker.color.g = 0.0
        obs_marker.color.b = 0.0 
        
        self.marker_pub.publish(obs_marker)
        
        
    def sensor_callback(self,msg):
        if len(msg.position) >= 3:
            self.current_JS.position = msg.position
        
    def control_loop(self):
        
        now = self.get_clock().now() # Figure out when now is (interesting sentence)
        delta_t = (now - self.last_time).nanoseconds / 1e9 # Figure out how long its been since last update
        if delta_t == 0: return 
        
        # Get the final goal
        final_goal = self.waypoints[self.current_idx]
        
        # self.q1 = self.current_JS.position[0]
        # self.q2 = self.current_JS.position[1]
        # self.q3 = self.current_JS.position[2]
        
        vel_1 = self.pid_1.update(final_goal.position[0],self.current_JS.position[0], delta_t)
        vel_2 = self.pid_2.update(final_goal.position[1],self.current_JS.position[1], delta_t)
        vel_3 = self.pid_3.update(final_goal.position[2],self.current_JS.position[2], delta_t)
        
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(vel_1), float(vel_2), float(vel_3), float(delta_t)]
        
        self.vel_pub.publish(cmd_msg)
        
        # # Base -> Joint 1 (Rotation Z) -> Link 1 Tip (Translation Z)
        # T_01 = kinematics.rot_z(self.q1) @ kinematics.trans(0, 0, self.L1)

        # # Link 1 -> Joint 2 (Rotation Y) -> Link 2 Tip (Translation X)
        # T_12 = kinematics.rot_y(self.q2) @ kinematics.trans(self.L2, 0, 0)

        # # Link 2 -> Joint 3 (Rotation Y) -> End Effector (Translation X)
        # T_23 = kinematics.rot_y(self.q3) @ kinematics.trans(self.L3, 0, 0)
        
        # # Position of Elbow (End of Link 1) 
        # pos_elbow_matrix = T_01
        # xyz_elbow = pos_elbow_matrix[:3, 3]

        # # Position of Wrist (End of Link 2)
        # pos_wrist_matrix = T_01 @ T_12
        # xyz_wrist = pos_wrist_matrix[:3, 3]

        # # Position of Tip (End Effector)
        # pos_tip_matrix = T_01 @ T_12 @ T_23
        # xyz_tip = pos_tip_matrix[:3, 3]
        
        err_1 = math.fabs(final_goal.position[0] - self.current_JS.position[0])
        err_2 = math.fabs(final_goal.position[1] - self.current_JS.position[1])
        err_3 = math.fabs(final_goal.position[2] - self.current_JS.position[2])
        
        is_close = err_1 < self.tolerance and err_2 < self.tolerance
        
        # start checking done after move time has finished
        
        if is_close:
            if self.stable_start_time is None:
                self.stable_start_time = now
            
            elapsed = (now - self.stable_start_time).nanoseconds / 1e9
            
            if elapsed > self.required_time:
                # switching targets
                print(f"Move {self.current_idx} Complete.")
                
                # 1. Save where we are NOW as the start for the NEXT move
                self.start_angles = [self.current_JS.position[0], self.current_JS.position[1], self.current_JS.position[2]]
                
                # 2. Reset timers
                self.move_start_time = now
                self.stable_start_time = None
                
                # 3. Increment Index
                self.current_idx += 1
                if self.current_idx >= len(self.waypoints):
                    self.current_idx = 0
                    print("Resetting to Waypoint 0")
        else:
            self.stable_start_time = None
        
        # msg = JointState()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = "world"
        
        # msg.name = ['joint1','joint2','joint3']
        # msg.position = [float(self.q1), float(self.q2), float(self.q3)]
        
        # self.joint_pub.publish(msg)
        
        # self.q1 = math.radians(float(input("new q1: ")))
        # self.q2 = math.radians(float(input('new q2: ')))
        # self.q3 = math.radians(float(input('new q3: ')))
        
        
        
        
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