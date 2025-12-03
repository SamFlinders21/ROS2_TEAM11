import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point

import math

# Create a node that will publish the joint states 
# References: 
# Python library: https://docs.python.org/3/library/functions.html 
# ROS2 Humble documentation: 
# RVIZ User Guide: https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html

# This file is kinda similar to the code I wrote for the mini project because it outputs the joint states. However,
# the way that it works is completely different from the mini project. 
# The way it works is that it takes the signals from the PID calculation from robot_controller.py and multiplies it by
# the time that it has been since the last time it got an update, and adds it to the angular velocity of the joint. 
# It then sends the velocity data back to robot_controller.py and the process starts again.
class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_limits_node')
            
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        # self.delta_t = 0
            
        self.last_cmd_time = self.get_clock().now()
            
        # subscribe to the pid control signal
        self.pid_subscription = self.create_subscription(
            Float64MultiArray,
            'joint_velocities',
            self.cmd_callback,
            10
        )
            
        self.publisher_ = self.create_publisher(JointState, '/joint_states',10)
        
    def cmd_callback(self,cmd_msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if dt > 0.1: 
            dt = 0.0 # Ignore jumps
            
        if len(cmd_msg.data) >= 3:
            
            
            
            self.last_cmd_time = self.get_clock().now()
            self.vel[0] = cmd_msg.data[0]
            self.vel[1] = cmd_msg.data[1]
            self.vel[2] = cmd_msg.data[2]
            # self.delta_t = cmd_msg.data[3]
            
            self.pos[0] += self.vel[0] * dt
            self.pos[1] += self.vel[1] * dt
            self.pos[2] += self.vel[2] * dt
        
        # self.ang += self.angVel * delta_t
        
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = ['joint1','joint2','joint3']
        msg.position = self.pos
        msg.velocity = self.vel
        self.publisher_.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    
    joint_limits_node = JointPublisher()
    
    try:
        rclpy.spin(joint_limits_node)
    except KeyboardInterrupt:
        pass
    
    joint_limits_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()