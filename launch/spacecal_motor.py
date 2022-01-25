# Copyright 2021 UC-Berkeley
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='JNode_MotorController',
            namespace='printerTesting',
            executable='motorRun',
            output='screen',
            emulate_tty=True,
            parameters=[{"Address": 14}],
            name="motor"
        ),
        Node(
            package='JNode_MotorController',
            namespace='printerTesting',
            executable='motorRun',
            output='screen',
            emulate_tty=True,
            parameters=[{"Address": 15}],
            name="motor2",
        ),
        Node(
            package='JNode_MotorController',
            namespace='printerTesting',
            executable='motorRun',
            output='screen',
            emulate_tty=True,
            parameters=[{"Address": 16}],
            name="motor3",
        ),
        Node(
            package='JNode_MotorController',
            namespace='printerTesting',
            executable='motorRun',
            output='screen',
            emulate_tty=True,
            parameters=[{"Address": 17}],
            name="motor4",
        ),
        # Spin motors
        Node(
            package='JNode_MotorController',
            namespace='printerTesting',
            executable='motorRun',
            output='screen',
            emulate_tty=True,
            parameters=[{"Address": 18}],
            remappings=[("setVelocity","setVelocityV1")],
            name="motorV1",
        ),
        Node(
            package='JNode_MotorController',
            namespace='printerTesting',
            executable='motorRun',
            output='screen',
            emulate_tty=True,
            parameters=[{"Address": 19}],
            remappings=[("setVelocity","setVelocityV2")],
            name="motorV2",
        ),
        Node(
            package='JNode_MotorController',
            namespace='printerTesting',
            executable='motorRun',
            output='screen',
            emulate_tty=True,
            parameters=[{"Address": 20}],
            remappings=[("setVelocity","setVelocityV3")],
            name="motorV3",
        ),
        Node(
            package='JNode_MotorController',
            namespace='printerTesting',
            executable='motorRun',
            output='screen',
            emulate_tty=True,
            parameters=[{"Address": 21}],
            remappings=[("setVelocity","setVelocityV4")],
            name="motorV4",
        ),
        Node(
            package='JNode_MotorController',
            namespace='printerTesting',
            executable='motorRun',
            output='screen',
            emulate_tty=True,
            parameters=[{"Address": 22}],
            remappings=[("setVelocity","setVelocityV5")],
            name="motorV5",
        )
    ])
