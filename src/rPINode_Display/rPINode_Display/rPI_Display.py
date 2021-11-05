import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import threading

mProcess = None
lightOn = False

# Looks for a video string to display, and displays it
class displayFunctionClass(Node):
    def __init__(self):
        super().__init__('ProjectorDisplaySubscriber')
        self.videoToPlaySubscriber = self.create_subscription(
            String, 'videoName', self.displayVideo, 10)
        self.videoToPlaySubscriber
        stayAlive = threading.Thread(target=self.stay_alive())
        stayAlive.daemon = True
        stayAlive.start()
    
    def displayVideo(self, msg):
        # First kill any current projection
        # subprocess.run(['export DISPLAY=":0"'], shell=True)
        os.environ['DISPLAY']=":0"
        global mProcess
        global lightOn
        if(mProcess is not None):
            mProcess.kill()
        subprocess.run(['ledOn'])
        lightOn = True
        # Now Project
        videoString = '/home/spacecal/test_video/' + msg.data
        mProcess = subprocess.Popen(
            ['mplayer', "-slave", "-quiet", videoString],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL)

    def stay_alive(self):
        global lightOn
        while rclpy.ok():
            if(mProcess.poll() is None and lightOn):
                lightOn = False
                subprocess.run(['ledZero'])

# Main function to start subscriber but also set display correctly
def main(args=None):
    # Set the proper OS variable to display on
    os.environ['DISPLAY']=":0"
    # Resets the display to resize correctly
    subprocess.run(['xset', 'dpms', 'force', 'off'])
    rclpy.init(args=args)
    subscriber = displayFunctionClass()
    rclpy.spin(subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()