#Generic node that will check for various key inputs and communicate them via String

import rclpy
from rclpy.node import Node
from pynput import keyboard
from std_msgs.msg import String
import time
class keyTalkerClass(Node):
    def __init__(self):
        super().__init__('keyTalker')
        self.publisher_ = self.create_publisher(String, 'keyinput',10)
        self.keyTalker()

    def keyTalker(self):
        while not rclpy.ok():
            with keyboard.Events() as events:
                event = events.get(1e6)
                if event.key == keyboard.KeyCode.from_char('w'):
                    self.publisher_.publish("w")
                    print("W")
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
