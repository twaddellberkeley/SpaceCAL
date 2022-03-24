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


# Generic node that will check for various key inputs and
# communicate them via String
# Author: Taylor Waddel
import rclpy
from rclpy.node import Node
from pynput import keyboard
from std_msgs.msg import String, Int32
import time
# Setting correct display
# import os
# os.environ.setdefault('DISPLAY', ':0')
# os.environ['DISPLAY'] = ":0"


# generic keytalker class
class guiDisplay(Node):
    def __init__(self):
        # Create publisher under keyTalker
        super().__init__('keyTalker')
        self.keyPublisher = self.create_publisher(String, 'keyinput', 10)
        self.velocitySpin = self.create_publisher(Int32, 'setVelocityV2', 10)
        self.videoChange = self.create_publisher(String, 'videoNameV2', 10)
        self.keyTalker()



def main(args=None):
    rclpy.init(args=args)
    publisher =  guiDisplay
    rclpy.spin(publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
