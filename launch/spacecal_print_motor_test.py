from launch import LaunchDescription
from launch_ros.actions import Node

####### This launch file is to test the motors to rotate the vile with the main logic pakg

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
            package='print_motor_controller',
            #namespace='test',
            output='screen',
            executable='print_motor_node',
            parameters=[{"motor_number": 0}, {"address": 18}],
            name='print_motor_0'
        ),
        Node(
            package='print_motor_controller',
            #namespace='test',
            output='screen',
            executable='print_motor_node',
            parameters=[{"motor_number": 1}, {"address": 19}],
            name='print_motor_1'
        ),
        Node(
            package='print_motor_controller',
            #namespace='test',
            output='screen',
            executable='print_motor_node',
            parameters=[{"motor_number": 2}, {"address": 20}],
            name='print_motor_2'
        ),
        Node(
            package='print_motor_controller',
            #namespace='test',
            output='screen',
            executable='print_motor_node',
            parameters=[{"motor_number": 3}, {"address": 21}],
            name='print_motor_3'
        ),
        Node(
            package='print_motor_controller',
            #namespace='test',
            output='screen',
            executable='print_motor_node',
            parameters=[{"motor_number": 4}, {"address": 22}],
            name='print_motor_4'
        )
    ])