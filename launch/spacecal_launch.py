from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='JNode_KeyboardInput',
            namespace='keyboard',
            executable='JNodeInput',
            output='screen',
            emulate_tty=True,
            name='sim'
        ),
        Node(
            package='JNode_MotorController',
            namespace='keyboard',
            executable='keyboardRun',
            output='screen',
            emulate_tty=True,
            name='sim'
        )        
    ])
