import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess

# odometría
def generate_launch_description():
	color_identification= Node(
			package = 'color_identification', #basic_comms
			executable = 'color_identification',
			output = 'screen'
	)

	video_node= Node(
		package = 'ros_deep_learning', #basic_comms
		executable = 'video_source.ros2.launch',
		output = 'screen'
		
	)
	
	

	micro_ros_node = ExecuteProcess(
        cmd=['gnome-terminal','--','ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0'],
        output='screen'
    )
 

	

		
	l_d = LaunchDescription([video_node])
	return l_d

