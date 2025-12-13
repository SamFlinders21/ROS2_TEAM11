import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robot_controller'
    urdf_file_name = 'robot.urdf'

    # 1. Locate the URDF file
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name)

    # Read the file content
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # NODE 1: Robot State Publisher
        # This is the "Translator". It takes your URDF and your Joint States
        # and turns them into 3D coordinates for RViz.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf_path]
        ),
        
        #NODE 2: Your Custom Controller
        # This replaces the 'joint_state_publisher_gui'.
        # It runs your Python script to generate the angles.
        Node(
            package='robot_controller',
            executable='run_controller', # Must match 'console_scripts' in setup.py
            name='my_custom_controller',
            output='screen'
        ),

        # NODE 3: RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])