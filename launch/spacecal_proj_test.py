from launch import LaunchDescription
from launch_ros.actions import Node

####### This launch file is to test the projecors with the main logic pakg

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main_logic',
            #namespace='test',
            output='screen',
            executable='main_service',
            name='main_service'
        ),
        Node(
            package='projector_exec',
            #namespace='test',
            output='screen',
            executable='proj_node',
            parameters=[{"projector_number": 0}],
            name='proj_0'
        ),
        Node(
            package='projector_exec',
            #namespace='test',
            output='screen',
            executable='proj_node',
            parameters=[{"projector_number": 1}],
            name='proj_1'
        ),
        Node(
            package='projector_exec',
            #namespace='test',
            output='screen',
            executable='proj_node',
            parameters=[{"projector_number": 2}],
            name='proj_2'
        ),
        Node(
            package='projector_exec',
            #namespace='test',
            output='screen',
            executable='proj_node',
            parameters=[{"projector_number": 3}],
            name='proj_3'
        ),
        Node(
            package='projector_exec',
            #namespace='test',
            output='screen',
            executable='proj_node',
            parameters=[{"projector_number": 4}],
            name='proj_4'
        )
    ])