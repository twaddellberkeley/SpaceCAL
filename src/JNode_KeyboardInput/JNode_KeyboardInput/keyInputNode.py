#Generic node that will check for various key inputs and communicate them via String

import rclpy
from rclpy.node import Node
from pynput import keyboard
from std_msgs.msg import String
import time
import sys
if sys.platform not in ('darwin', 'win32'):
    import os

    os.environ.setdefault('DISPLAY', ':0')
class keyTalkerClass(Node):
    def __init__(self):
        super().__init__('keyTalker')
        self.publisher_ = self.create_publisher(String, 'keyinput',10)
        self.get_logger().info("W")
        self.keyTalker()

    def keyTalker(self):
        self.get_logger().info("W2")
        while rclpy.ok():
            with keyboard.Events() as events:
                event = events.get(1e6)
                self.get_logger().info("event")
                if event.key == keyboard.KeyCode.from_char('w'):
                    self.publisher_.publish("w")
                    self.get_logger().info("W")
                    time.sleep(.1)
                elif event.key == keyboard.KeyCode.from_char('s'):
                    self.publisher_.publish("s")
                    time.sleep(.1)


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
