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
class keyTalkerClass(Node):
    def __init__(self):
        # Create publisher under keyTalker
        super().__init__('keyTalker')
        self.keyPublisher = self.create_publisher(String, 'keyinput', 10)
        self.velocitySpin = self.create_publisher(Int32, 'setVelocityV2', 10)
        self.videoChange = self.create_publisher(String, 'videoName', 10)
        self.keyTalker()

    def on_press(self, key):
        try:
            # Create a message of type string, and pass it along if its w or s
            msg = String()
            inputKey = str(key.char)
            intMsg = Int32()
            if (inputKey == "w"):
                msg.data = inputKey
                self.keyPublisher.publish(msg)
            elif (inputKey == "s"):
                msg.data = str(key.char)
                self.keyPublisher.publish(msg)
            elif (inputKey == "h"):
                msg.data = str(key.char)
                self.keyPublisher.publish(msg)
            elif (inputKey == "j"):
                msg.data = inputKey
                self.keyPublisher.publish(msg)
            elif (inputKey == "m"):
                msg.data = str(key.char)
                self.keyPublisher.publish(msg)
            elif (inputKey == "t"):
                msg.data = "test.mkv"
                self.videoChange.publish(msg)
            elif (inputKey == "y"):
                msg.data = "out.mp4"
                self.videoChange.publish(msg)
            elif (inputKey == "u"):
                msg.data = "dot.mp4"
                self.videoChange.publish(msg)
            elif (inputKey == "p"):
                intMsg.data = 5
                self.velocitySpin.publish(intMsg)
            time.sleep(.5)
        # If it nots a char, we error
        except AttributeError:
            print('special key {0} pressed'.format(
                key))

    # Keep logging for keys while ros is alive
    def keyTalker(self):
        while rclpy.ok():
            with keyboard.Listener(
                    on_press=self.on_press) as listener:
                listener.join()


def main(args=None):
    rclpy.init(args=args)
    publisher = keyTalkerClass()
    rclpy.spin(publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
