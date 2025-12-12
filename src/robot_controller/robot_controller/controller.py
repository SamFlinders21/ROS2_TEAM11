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
from visualization_msgs.msg import Marker
from robot_controller.planner import RRTplanner, GetJointLocations, Kinematics


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
    
    
class RobotController(Node):
    def __init__(self):
        super().__init__('controller')
        self.timer = self.create_timer(1/90,self.control_loop)
        
        self.q1 = math.radians(0)
        self.q2 = math.radians(0)
        self.q3 = math.radians(0)
        
        self.L1 = .2
        self.L2 = .3
        self.L3 = .1        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        #################################################
        self.pid_1 = PID_controller(Kp=2,Ki=0,Kd=0.20)
        self.pid_2 = PID_controller(Kp=2,Ki=0,Kd=0.10)
        self.pid_3 = PID_controller(Kp=2,Ki=0,Kd=0.10)
        #################################################
        
        self.last_time = self.get_clock().now()
        self.vel_pub = self.create_publisher(Float64MultiArray, 'joint_velocities', 10)
        
        # Indexing/tolerance
        self.current_idx = 0
        self.tolerance = math.radians(1)
        self.required_time = 0.001 # How long the arm is required to be within tolerance
        self.stable_start_time = None
        self.move_start_time = self.get_clock().now()
        self.lap_time = 0
        
        # Current Angle
        self.current_JS = JointState()
        self.name = ['joint1','joint2','joint3']
        self.current_JS.position = [0.0,0.0,0.0]
        
        start_conf = [math.radians(0), math.radians(-90), math.radians(-90)] # Home
        pick_conf = [math.radians(90), math.radians(-90), math.radians(-90)]

        # define obstacles
        self.obs_x = -0.3
        self.obs_y = -0.3
        self.obs_z = 0.20
        self.obs_radius = .25
        obstacles = [[self.obs_x, self.obs_y, self.obs_z, self.obs_radius]]
        
        # Display Obstacle 
        self.marker_pub = self.create_publisher(Marker, '/spheres', 10)
        self.publish_sphere()
        
        # Display Collision Spheres
        self.collision_debug = self.create_publisher(Marker, '/arm_collision_spheres', 10)
        
        # Limits (from urdf)
        joint_limits = {
            'joint1': (-3.14, 3.14),
            'joint2': (-3.14, 3.14),
            'joint3': (-3.14, 3.14)
        }
        
        self.viz_pub = self.create_publisher(Marker, 'rrt_tree', 10)

        # initialize the planner
        self.get_logger().info('Starting RRT Planner...')
        self.rrt = RRTplanner(start_conf,pick_conf, obstacles, joint_limits, self.viz_pub)
        
        # run the planner
        path_list = self.rrt.plan()
        
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
        
        # Track where we started the current motion
        self.start_angles = [0.0, 0.0, 0.0]
        
        # Creates a subscriber node that listens to the joint_states topic, 
        # expecting a "JointState" message. Each time that it receives one, 
        # it calls the self.sensor_callback function
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.sensor_callback,
            10
        )
        
        self.moving_forward = True

        
        
    def sensor_callback(self,msg):
        if len(msg.position) >= 3:
            self.current_JS.position = msg.position
        
    def control_loop(self):
        
        now = self.get_clock().now() # Figure out when now is (interesting sentence)
        delta_t = (now - self.last_time).nanoseconds / 1e9 # Figure out how long its been since last update
        if delta_t == 0: return 
        
        # Get the final goal
        final_goal = self.waypoints[self.current_idx]
        
        vel_1 = self.pid_1.update(final_goal.position[0],self.current_JS.position[0], delta_t)
        vel_2 = self.pid_2.update(final_goal.position[1],self.current_JS.position[1], delta_t)
        vel_3 = self.pid_3.update(final_goal.position[2],self.current_JS.position[2], delta_t)
        
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(vel_1), float(vel_2), float(vel_3), float(delta_t)]
        
        current_joints = [float(self.current_JS.position[0]),float(self.current_JS.position[1]),float(self.current_JS.position[2])]
        
        self.publish_arm_collision_model(current_joints)
        
        self.vel_pub.publish(cmd_msg)
        
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
                if self.moving_forward:
                    self.current_idx += 1
                else:
                    self.current_idx -= 1
                    
                if self.current_idx >= len(self.waypoints) and self.moving_forward:
                    

                    user_input = input('Type "r" to return to start. ')
                    
                    if user_input == 'r':
                        self.moving_forward = False
                        self.current_idx = len(self.waypoints) - 1
                        print('Returning to Start...')
                    
                    else:
                        print('Invalid Input. Stopping.')
                            
                elif not self.moving_forward and self.current_idx < 0:
                    print('Returned to Start!')

                    user_input = input('Enter one of the following: \n'
                                       '"new" to generate a new path \n'
                                       '"old" to repeat the same path \n')
                    
                    if user_input == 'new':
                        self.moving_forward = True
                        self.current_idx = 0
                        print('Generating new RRT Plan...')
                        path_list = self.rrt.plan()
                        self.waypoints = []
                        if path_list is not None:
                            for point in path_list:
                                wp = JointState()
                                wp.name = ['joint1','joint2','joint3']
                                wp.position = point
                                self.waypoints.append(wp)
                        
                    elif user_input == 'old':
                        self.moving_forward = True
                        self.current_idx = 0
                        print('Moving to Goal with same Plan...')
        else:
            self.stable_start_time = None
        
        
    def publish_arm_collision_model(self, current_joints):
        
        class FakeNode: # creates a dummy object because GetJointLocations expects an object with a .joints attribute
            def __init__(self, j): self.joints = j
            
        node = FakeNode(current_joints)

        # find the sphere centers
        
        joint_locations = GetJointLocations(node)
        p_elbow = joint_locations.elbow()
        p_wrist = joint_locations.wrist()
        p_tip = joint_locations.tip()
        
        mid_link_2 = (p_elbow + p_wrist) / 2
        mid_link_3 = (p_wrist + p_tip) / 2
        
        sphere_centers = [p_elbow, mid_link_2, p_wrist, mid_link_3, p_tip]
        
        # set up the marker
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg() 
        
        marker.ns = "arm_spheres"   
        marker.id = 999             
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        
        # set the scale properly
        radius = 0.05 
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = radius * 2.0
        
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.6 
        
        # add the points
        for center in sphere_centers:
            p = Point()
            p.x = float(center[0])
            p.y = float(center[1])
            p.z = float(center[2])
            marker.points.append(p)
            
        self.collision_debug.publish(marker)
        
    def publish_sphere(self):
        obs_marker = Marker()
        obs_marker.header.frame_id = "base_link"
        obs_marker.header.stamp = self.get_clock().now().to_msg()
        obs_marker.type = Marker.SPHERE
        obs_marker.action = Marker.ADD
        obs_marker.ns = "obstacles"
        obs_marker.id = 1

        obs_marker.pose.position.x = self.obs_x
        obs_marker.pose.position.y = self.obs_y
        obs_marker.pose.position.z = self.obs_z
        
        obs_marker.pose.orientation.x = 0.0
        obs_marker.pose.orientation.y = 0.0
        obs_marker.pose.orientation.z = 0.0
        obs_marker.pose.orientation.w = 1.0
        
        obs_marker.scale.x = 2 * self.obs_radius
        obs_marker.scale.y = 2 * self.obs_radius
        obs_marker.scale.z = 2 * self.obs_radius
        
        obs_marker.color.a = 1.0
        obs_marker.color.r = 1.0
        obs_marker.color.g = 0.0
        obs_marker.color.b = 0.0 
        
        self.marker_pub.publish(obs_marker)
        
        
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