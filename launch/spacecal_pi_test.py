from launch import LaunchDescription
from launch_ros.actions import Node


### This lauch file is to test the commands between the pi and the main logic pakg
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
            package='pi_video_controller',
            #namespace='test',
            output='screen',
            executable='pi_video_node',
            parameters=[{"pi_number": 0}],
            name='pi_video_0'
        ),
        Node(
            package='pi_video_controller',
            #namespace='test',
            output='screen',
            executable='pi_video_node',
            parameters=[{"pi_number": 1}],
            name='pi_video_1'
        ),
        Node(
            package='pi_video_controller',
            #namespace='test',
            output='screen',
            executable='pi_video_node',
            parameters=[{"pi_number": 2}],
            name='pi_video_2'
        ),
        Node(
            package='pi_video_controller',
            #namespace='test',
            output='screen',
            executable='pi_video_node',
            parameters=[{"pi_number": 3}],
            name='pi_video_3'
        ),
        Node(
            package='pi_video_controller',
            #namespace='test',
            output='screen',
            executable='pi_video_node',
            parameters=[{"pi_number": 4}],
            name='pi_video_4'
        )
    ])