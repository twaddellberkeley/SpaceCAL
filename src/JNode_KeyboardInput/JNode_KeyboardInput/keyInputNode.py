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
    
    def on_press(self,key):
        try:
            self.get_logger().info("event")
            inputKey = str(key.char)
            if (inputKey == "w"):
                msg.data = inputKey
                self.publisher_.publish(msg)
                self.get_logger().info("W")
            elif (inputKey == "s"):
                msg.data = str(key.char)
                self.publisher_.publish(msg)
            time.sleep(.5)
        except AttributeError:
            print('special key {0} pressed'.format(
                key))   

    def keyTalker(self):
        self.get_logger().info("W2")
        msg = String()
        
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
