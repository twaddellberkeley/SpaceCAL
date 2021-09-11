#Generic node that will check for various key inputs and communicate them via String

import rospy
import st
from pynput import keyboard
from std_msgs.msg import String
import time
def keyTalker():
    pub = rospy.Publisher('keyInput',String,queue_size=10)
    rospy.init_node('keyTalker',anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        with keyboard.Events() as events:
            event = events.get(1e6)
            if event.key == keyboard.KeyCode.from_char('w'):
                pub.publish("w")
                time.sleep(.1)
            elif event.key == keyboard.KeyCode.from_char('s'):
                pub.publish("s")
                time.sleep(.1)


if __name__ == '__main__':
    try:
        keyTalker()
    except rospy.ROSInterruptException:
        pass