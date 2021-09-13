from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='JNode_KeyboardInput',
            namespace='keyboard',
            executable='JNodeInput',
	    output='screen',
            name='sim'
        )
    ])
