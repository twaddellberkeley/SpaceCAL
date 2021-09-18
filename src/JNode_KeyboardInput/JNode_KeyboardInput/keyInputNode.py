# Generic node that will check for various key inputs and communicate them via String
# Author: Taylor Waddell
# Copyright 2021 MIT Opensource License
import rclpy
from rclpy.node import Node
from pynput import keyboard
from std_msgs.msg import String
import time
import sys
# Setting correct display
if sys.platform not in ('darwin', 'win32'):
    import os
    os.environ.setdefault('DISPLAY', ':0')


# generic keytalker class
class keyTalkerClass(Node):
    def __init__(self):
        # Create publisher under keyTalker
        super().__init__('keyTalker')
        self.publisher_ = self.create_publisher(String, 'keyinput', 10)
        self.keyTalker()

    def on_press(self, key):
        try:
            # Create a message of type string, and pass it along if its w or s
            msg = String()
            inputKey = str(key.char)
            if (inputKey == "w"):
                msg.data = inputKey
                self.publisher_.publish(msg)
            elif (inputKey == "s"):
                msg.data = str(key.char)
                self.publisher_.publish(msg)
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
