from launch import LaunchDescription
from launch_ros.actions import Node

####### This launch file is to test the motors to rotate the vile with the main logic pakg

def generate_launch_description():
    return LaunchDescription([
        #############*********** Level Motors **************######################
        Node(
            package='level_motor_controller',
            #namespace='test',
            output='screen',
            executable='level_motor_node',
            parameters=[{"motor_number_0": 0}, {"address_14": 14}, {"motor_number_1": 1}, {"address_15": 15},{"motor_number_2": 2}, {"address_16": 16},{"motor_number_3": 3}, {"address_17": 17}],
            name='level_motor_controller'
        ),

        
        ###############****************** Printer Motors ****************#################
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