from launch import LaunchDescription
from launch_ros.actions import Node

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
            executable='proj_node_0',
            # name='proj_0'
        ),
        Node(
            package='projector_exec',
            #namespace='test',
            output='screen',
            executable='proj_node_1',
            name='proj_1'
        ),
        Node(
            package='projector_exec',
            #namespace='test',
            output='screen',
            executable='proj_node_2',
            name='proj_2'
        ),
        Node(
            package='projector_exec',
            #namespace='test',
            output='screen',
            executable='proj_node_3',
            name='proj_3'
        ),
        Node(
            package='projector_exec',
            #namespace='test',
            output='screen',
            executable='proj_node_4',
            name='proj_4'
        )
    ])