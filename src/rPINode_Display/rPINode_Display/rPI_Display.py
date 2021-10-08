import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

mProcess = None


# Looks for a video string to display, and displays it
class displayFunctionClass(Node):
    def __init__(self):
        super().__init__('ProjectorDisplaySubscriber')
        self.videoToPlaySubscriber = self.create_subscription(
            String, 'videoName', self.displayVideo, 10)
        self.videoToPlaySubscriber
    
    def displayVideo(self, msg):
        # First kill any current projection
        # subprocess.run(['export DISPLAY=":0"'], shell=True)
        os.environ['DISPLAY']=":0"
        global mProcess
        if(mProcess is not None):
            mProcess.kill()
        # Now Project
        videoString = '/home/spacecal/test_video/' + msg.data
        mProcess = subprocess.Popen(
            ['mplayer', videoString],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL)


# Main function to start subscriber but also set display correctly
def main(args=None):
    # Set the proper OS variable to display on
    os.environ['DISPLAY']=":0"
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