import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import multiprocessing
import os
import time

mProcess = None 
stayAlive = None

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
        os.environ['DISPLAY']=":0"
        global stayAlive
        global mProcess
        print("DISPLAYING\n")
        if(mProcess != None and mProcess.poll() is None):
            # Need to kill thread
            stayAlive.terminate()
            mProcess.kill()
        # Now Project from givin string
        videoString = '/home/spacecal/test_video/' + msg.data
        # Turn our LED on to project
        subprocess.run(['ledOn'])
        mProcess = subprocess.Popen(
            ['mplayer', "-slave", "-quiet", videoString],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL)
        time.sleep(.5)
        # Create multiprocess to turn of projector when done
        stayAlive = multiprocessing.Process(target=self.kill_me, args=mProcess)
        stayAlive.start()

    def kill_me(self, mProcess):
        # Check to see if our projection process is running
        print( "CHECK PROCESS: " + str(mProcess.poll()))
        mProcess.wait()
        print("Killing\n")
        # When dead turn off projector
        subprocess.run(['ledZero'])

# Main function to start subscriber but also set display correctly
def main(args=None):
    print("STARTING")
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