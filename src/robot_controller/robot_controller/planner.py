import numpy as np
import random
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.clock import Clock

class Node:
    def __init__(self, joints):
        self.joints = joints
        self.parent = None
        
class Kinematics:    
    @staticmethod    
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
    @staticmethod
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
    @staticmethod
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
    @staticmethod
    def trans(x,y,z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])
    
class RRTplanner:
    def __init__(self,start_conf,goal_conf,obstacles, joint_limits, viz_pub=None):
        self.start = Node(start_conf)
        self.goal = Node(goal_conf)
        self.obstacles = obstacles
        self.limits = joint_limits
        self.node_list = [self.start]
        self.step_size = 0.1 
        self.viz_pub = viz_pub
        
        self.L1 = .2
        self.L2 = .3
        self.L3 = .1 
        
        self.marker = Marker()
        self.marker.header.frame_id = "base_link" 
        self.marker.type = Marker.LINE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05 
        self.marker.color.a = 0.5
        self.marker.color.g = 1.0 
        
        
        
    def plan(self):
        
        print("Starting RRT Plan...") 
        
        # Inside plan(self):
        print("Checking Start and Goal...")
        
        if not self.is_collision_free(self.start):
            print("ERROR: Start configuration is in collision!")
            return None
        if not self.is_collision_free(self.goal):
            print("ERROR: Goal configuration is in collision! (Check floor or obstacles)")
            return None
        
        
        # Try to find a path a set number of times
        for i in range(5000):
            
            print(f"We are in iteration #{i}!")
            
            # pick a random node
            rnd_node = self.get_random_node()
            
            # find the closest existing node
            nearest_node = self.get_nearest_node(self.node_list,rnd_node)
            
            # steer towards the random node by the set step size
            new_node = self.steer(nearest_node, rnd_node, self.step_size)
            

            
            # check to see if the new configuration is safe (no collision)
            if self.is_collision_free(new_node):
                self.node_list.append(new_node)
                self.publish_tree(new_node)
                
                # check if we are close enough to the goal
                if self.calc_dist(new_node, self.goal) < 0.2:
                    print(f"Goal Reached in {i} iterations!")
                    self.goal.parent = new_node
                    
                    # generate the random path
                    final_path = self.generate_final_path(self.goal)
                    
                    # refine the path via smoothing up to 5 times
                    for _ in range(5):
                        old_len = len(final_path)
                        final_path = self.path_smoother(final_path)
                    
                        # if the latest iteration does do anything, stop smoothing
                        if len(final_path) == old_len: break
                        
                    return final_path
                
            if i % 50 == 0:
                dist_to_goal = self.calc_dist(nearest_node, self.goal)
                print(f"Iter {i}: Tree size {len(self.node_list)}. Nearest dist to goal: {dist_to_goal:.4f}")
                
        return None # couldnt find a path
    
    def steer(self, from_node, to_node, step_size):
        # This function will increment towards the node that we told it to go to    
        start_joints = np.array(from_node.joints)
        target_joints = np.array(to_node.joints)
        
        joint_vector = target_joints - start_joints
        norm = np.linalg.norm(joint_vector)
        
        #dont divide by zero
        if norm < 1e-6: return to_node
        
        # normalize
        unit_vector = joint_vector / norm
        move_dist = min(norm, step_size) # allows it to reach its target even if its closer than the step size
        
        new_joints = start_joints + (move_dist * unit_vector)
        
        new_joints_list = new_joints.tolist()
        
        new_node = Node(new_joints_list)
        new_node.parent = from_node
        
        return new_node
        
    
    def get_nearest_node(self,node_list,rnd_node):
        
        min_dist = float('inf')
        nearest_node = None
        
        for node in node_list:

            dist = self.calc_dist(node,rnd_node)
        
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
                
        return nearest_node
            
    def get_random_node(self):
        if random.random() < 0.1:
            return Node(list(self.goal.joints))
        
        q1 = random.uniform(self.limits['joint1'][0], self.limits['joint1'][1])
        q2 = random.uniform(self.limits['joint2'][0], self.limits['joint2'][1])
        q3 = random.uniform(self.limits['joint3'][0], self.limits['joint3'][1])
        
        return Node([q1,q2,q3])
    
    def calc_dist(self, node1, node2):
        dist = np.linalg.norm(np.array(node1.joints) - np.array(node2.joints))
        return dist
    
    def is_collision_free(self,node):
        joint_locations = GetJointLocations(node)
        p_elbow = joint_locations.elbow()
        p_wrist = joint_locations.wrist()
        p_tip = joint_locations.tip()
        
        # Create more points between the joints to prevent anything from slipping through
        mid_link_2 = (p_elbow + p_wrist) / 2
        mid_link_3 = (p_wrist + p_tip) / 2
        
        # Create a list of points 
        arm_points = [
            p_elbow,
            mid_link_2,
            p_wrist,
            mid_link_3,
            p_tip
        ]
        
        arm_radius = 0.05
        
        for obs in self.obstacles:
            obs_pos = np.array([obs[0], obs[1], obs[2]])
            obs_radius = obs[3]
            
            for point in arm_points:
                dist = np.linalg.norm(point - obs_pos)
                
                if dist < (arm_radius + obs_radius):
                    return False # me when collision detected
            
        if p_tip[2] < 0.0:
            return False # me when floor detected
        
        return True # me when safe
    
    def publish_tree(self,new_node):
        if self.viz_pub is None or new_node.parent is None:
            return
        
        # Get XYZ of Parent
        loc_parent = GetJointLocations(new_node.parent)
        p1 = Point()
        p1.x, p1.y, p1.z = loc_parent.tip()

        # Get XYZ of New Node
        loc_child = GetJointLocations(new_node)
        p2 = Point()
        p2.x, p2.y, p2.z = loc_child.tip()
        
        self.marker.points.append(p1)
        self.marker.points.append(p2)
        
        self.viz_pub.publish(self.marker)

    def generate_final_path(self,goal_node):
        path = []
        current_node = goal_node
        
        while current_node is not None:
            path.append(current_node.joints)
            current_node = current_node.parent
            
        # return the path list (but reversed because we started with the goal)
        return path[::-1]
    
    
    def path_smoother(self,path_list):
        print('Waypoints received, smoothing path...')
        if len(path_list) < 3:
            return path_list
        
        smoothed_path = [path_list[0]]
        idx = 0
        
        while idx < len(path_list) - 1:
            # look ahead from the end of the list back to the current node
            found_shortcut = False
            
            # iterate backwards to find the furthest possible reachable node
            for check_index in range(len(path_list) - 1, idx, -1):
                
                # check if we can go straight from current to check index
                if self.segment_collision_check(path_list[idx], path_list[check_index]):
                    # if the shortcut is safe, add that target node and jump the idx forward
                    smoothed_path.append(path_list[check_index])
                    idx = check_index
                    found_shortcut = True
                    break
                
                if not found_shortcut:
                    smoothed_path.append(path_list[idx + 1])
                    idx += 1
                    
        print(f'Path smoothed from {len(path_list)} segments to {len(smoothed_path)} segments.')
        return smoothed_path
    
    def segment_collision_check(self, start_conf, end_conf):
        start_joints = np.array(start_conf)
        end_joints = np.array(end_conf)
        
        vector = end_joints - start_joints
        distance = np.linalg.norm(vector)
        
        step_size = 0.1
        num_steps = int(distance / step_size)
        
        # if the points are really close, just check the end
        if num_steps < 2:
            return True
        
        unit_vector = vector/distance
        
        # walk along the line
        for i in range(1,num_steps):
            #calculate intermediate joint config
            step_joints = start_joints + (unit_vector * (i * step_size))
            
            # make a temp node to check collision
            temp_node = Node(step_joints.tolist())
            
            # now check that node for collision
            if not self.is_collision_free(temp_node):
                return False
            
        return True
    
class GetJointLocations:
    def __init__(self,node):
        self.L1 = .2
        self.L2 = .3
        self.L3 = .1 
        self.base_height = 0.05
        
        self.q1 = node.joints[0]
        self.q2 = node.joints[1] - (np.pi / 2)
        self.q3 = node.joints[2]
        
        self.T_base_offset = Kinematics.trans(0, 0, self.base_height)
        self.T_01 = self.T_base_offset @ Kinematics.rot_z(self.q1) @ Kinematics.trans(0, 0, self.L1) # Get rotation from Base to elbow
        self.T_12 = Kinematics.rot_y(self.q2) @ Kinematics.trans(self.L2, 0, 0) # rotation from elbow to wrist
        self.T_23 = Kinematics.rot_y(self.q3) @ Kinematics.trans(self.L3, 0, 0) # rotation from wrist to tip
        
    def elbow(self):
        # Position of Elbow (End of Link 1) 
        pos_elbow_matrix = self.T_01
        xyz_elbow = pos_elbow_matrix[:3, 3]
        return xyz_elbow
        
    def wrist(self):
        # Position of Wrist (End of Link 2)
        pos_wrist_matrix = self.T_01 @ self.T_12
        xyz_wrist = pos_wrist_matrix[:3, 3]
        return xyz_wrist
        
    def tip(self):
        # Position of Tip (End Effector)
        pos_tip_matrix = self.T_01 @ self.T_12 @ self.T_23
        xyz_tip = pos_tip_matrix[:3, 3]
        return xyz_tip