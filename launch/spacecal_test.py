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
            package='jetsonDisplay',
            namespace='printerTesting',
            executable='guiMain',
            output='screen',
            emulate_tty=True,
            name='gui'
        ),
        Node(
            package='jetsonMainLogic',
            namespace='printerTesting',
            executable='printController',
            output='screen',
            emulate_tty=True,
            name='control'
        ),
        Node(
            package='bno055_imu_pub',
            namespace='printerTesting',
            executable='imu_node',
            output='screen',
            emulate_tty=True,
            name='imu'
        ),
        Node(
            package='bag_recorder_cpp',
            namespace='printerTesting',
            executable='topic_recorder',
            output='screen',
            emulate_tty=True,
            name='bag_recorder'
        )
    ])
