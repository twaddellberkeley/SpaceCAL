import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import os

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
        global mProcess
        if(mProcess != None and mProcess.poll() is None):
            #Need to kill thread
            stayAlive.stop()
            mProcess.kill()
        # Now Project from givin string
        videoString = '/home/spacecal/test_video/' + msg.data
        mProcess = subprocess.Popen(
            ['mplayer', "-slave", "-quiet", videoString],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL)
        # Create thread that is watches if projection should be alive
        # However only create if there is not another watcher
        global stayAlive
        stayAlive = customThread(target=self.kill_me)
        stayAlive.daemon = True
        stayAlive.start()
        # Turn our LED on to project
        subprocess.run(['ledOn'])

    def kill_me(self):
        global mProcess
        # Check to see if our projection process is running
        while (mProcess.poll() is None): pass
        # When dead turn off projector
        subprocess.run(['ledZero'])

class customThread(threading.Thread()):
    # Class to be able to stop thread.
    def __init__(self,  *args, **kwargs):
        super(customThread, self).__init__(*args, **kwargs)
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

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