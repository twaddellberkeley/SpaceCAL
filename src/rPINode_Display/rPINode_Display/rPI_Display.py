import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import os

mProcess = None 

# Looks for a video string to display, and displays it
class displayFunctionClass(Node):
    def __init__(self):
        super().__init__('ProjectorDisplaySubscriexiexitber')
        # Create our subscriber node
        self.videoToPlaySubscriber = self.create_subscription(
            String, 'videoName', self.displayVideo, 10)
        self.videoToPlaySubscriber
        # Starting a loop to detect if the projector should be on or off
    
    def displayVideo(self, msg):
        # First kill any current projection
        # subprocess.run(['export DISPLAY=":0"'], shell=True)
        os.environ['DISPLAY']=":0"
        global mProcess
        global lightOn
        if(mProcess != none and mProcess.poll() is None):
            mProcess.kill()
        subprocess.run(['ledOn'])
        # Now Project
        videoString = '/home/spacecal/test_video/' + msg.data
        mProcess = subprocess.Popen(
            ['mplayer', "-slave", "-quiet", videoString],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL)
        stayAlive = threading.Thread(target=self.stay_alive)
        stayAlive.daemon = True
        stayAlive.start()

    def kill_me():
        global mProcess
        while (mProcess.poll() is None): pass
        subprocess.run(['ledZero'])
        

# Main function to start subscriber but also set display correctly
def main(args=None):
    # Set the proper OS variable to display on
    os.environ['DISPLAY']=":0"
    # Resets the display to resize correctly
    subprocess.run(['xset', 'dpms', 'force', 'off'])
    subprocess.run(['ledZero'])
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