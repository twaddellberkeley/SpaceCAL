import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import multiprocessing
import os
import time

mProcess = None 
stayAlive = None
cPID = None

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
        global cPID
        print("DISPLAYING\n")
        if(stayAlive != None and stayAlive.is_alive()):
            # Need to kill thread
            print("HAPPENING\n")
            stayAlive.terminate()
            if (cPID != None):
                os.kill(cPID, 9)
        # Now Project from givin string
        sharedVal = multiprocessing.Value('i', cPID)
        videoString = '/home/spacecal/test_video/' + msg.data
        stayAlive = multiprocessing.Process(target=self.kill_me, args=(videoString,cPID,))
        stayAlive.daemon=True
        stayAlive.start()

    def kill_me(self, videoString, cPID):
        # Turn our LED on to project
        subprocess.run(['ledOn'])
        #subprocess.call(["mplayer", "-slave", "-quiet", videoString],
        #stderr=subprocess.DEVNULL,
        #stdout=subprocess.DEVNULL)
        cPID = subprocess.Popen(
            ['mplayer', "-slave", "-quiet", videoString],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL).pid
        cPID = None
        #mProcess.wait()
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