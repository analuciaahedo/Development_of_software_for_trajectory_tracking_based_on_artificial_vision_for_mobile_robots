import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odometria',
            executable='odometria',
            name='odometria',
            output='screen'
        ),
        Node(
            package='traffic_sign_recognition',
            executable='traffic_sign_recognition_node',
            name='traffic_sign_recognition',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='robot_control_node',
            name='robot_control',
            output='screen'
        ),
    ])
    
    l_d = LaunchDescription([odometriaaa_node, teleop_node, ros_bag_node])
	return l_d
