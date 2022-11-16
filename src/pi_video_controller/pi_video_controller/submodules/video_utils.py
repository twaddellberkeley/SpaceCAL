
import subprocess
import multiprocessing
from threading import Thread
import os
import time
import psutil

mProcess = None
stayAlive = None

started = False
VIDEO_DIR = '/home/spacecal/pi_videos/'
QUEUE_VIDEO_FILE = '/home/spacecal/pi_queue_videos/pi_queue.txt'

# Looks for a video string to display, and displays it

##################################################################################################################################################
class VlcControl:
    def __init__(self, logger):
        self._logger = logger
        self.videoProcess = None
        # Set the proper OS variable to display on
        os.environ['DISPLAY'] = ":0"
        # xprocess = subprocess.run(["sudo", "systemctl", "restart", "xserver.service"])
        # if xprocess.returncode != 0:
        #     self._logger.error("XSERVER.SERVICE Could not be restarted")
        
        if subprocess.run(["xrandr", "-o", "right"]).returncode != 0:
            self._logger.error("XRAND Could not be process")

        if subprocess.run(['xset', 'dpms', 'force', 'off', 's', 'off']).returncode != 0:
            self._logger.error("XSET Could not be process")
        
        self._logger.info("Finished initializing video controller")

    def playVideo(self, video):
        assert type(video) == type(""), "[playVideo] video name must be a string"
        
        if self.isVideoPlaying():
            self._logger.info("[playVideo]: stopping a video that was playig")
            self.stopVideo()
            
        # subprocess.run("ledOn")
        # Now Project from givin string
        os.environ['DISPLAY'] = ":0"
        videoString = VIDEO_DIR + video
        self.videoProcess = subprocess.Popen(
            args=['vlc-pi', "-I", "dummy", "-f", "--repeat",
                  "--no-audio", "--no-osd", videoString, "vlc://quit"],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL)
        self._logger.info("self.videoProcess.pid: %d\n" %(self.videoProcess.pid))
        killerProcess = multiprocessing.Process(
            target=self.kill_me, args=(self.videoProcess.pid,))
        killerProcess.start()

    def kill_me(self, pid):
        print(pid)
        process = psutil.Process(pid)
        while (process.status() != psutil.STATUS_ZOMBIE):
            time.sleep(1)
            # print(mProcess)
        print("Killing\n")
        # When dead turn off projector
        subprocess.run(['ledZero'])

    def stopVideo(self):
        time_to_wait = 10 
        waiting = 0
        if self.isVideoPlaying():
            self.videoProcess.terminate()
            while self.isVideoPlaying and time_to_wait < waiting:
                time.sleep(0.1)
                waiting += 1
                self._logger.warning("[stopVideo()]: Video process not terminate yet")
            if waiting == time_to_wait:
                self._logger.error("[stopVideo()]: Could not stop process")
                return
            self._logger.info("[stopVideo()]: Video process return code: %s\n" % ("isNone" if self.videoProcess == None else "isNotNone"))
        else:
            self._logger.warning("[stopVideo()]: There is no video process running")


    def isVideoPlaying(self):
        if self.videoProcess == None:
            self._logger.info("[isVideoPlaying]: videoProcess is None")
            return False
        elif self.videoProcess.poll() != None:
            return False
        return True

        



##################################################################################################################################################

def displayVideo( msg):
    if(msg.data == "PRINT"):
        subprocess.run(['ledOn'])
        return
    # First kill any current projection
    print(msg.data)
    os.environ['DISPLAY'] = ":0"
    global mProcess
    global stayAlive
    print("DISPLAYING\n")
    if(mProcess != None and mProcess.poll() is None):
        # Need to kill thread
        stayAlive.terminate()
        mProcess.kill()
    if(msg.data == "EXIT"):
        subprocess.run(['ledZero'])
        return
    # Now Project from givin string
    videoString = '/home/spacecal/test_video/' + msg.data
    #
    mProcess = subprocess.Popen(
        args=['vlc-pi', "-I", "dummy", "-f", "--repeat",
                "--no-audio", "--no-osd", videoString, "vlc://quit"],
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL)
    # Create multiprocess to turn of projector when done
    stayAlive = multiprocessing.Process(
        target=kill_me, args=(mProcess.pid,))
    stayAlive.start()

def kill_me(pid):
    print(pid)
    process = psutil.Process(pid)
    while (process.status() != psutil.STATUS_ZOMBIE):
        pass
        # print(mProcess)
    print("Killing\n")
    # When dead turn off projector
    subprocess.run(['ledZero'])


def reboot():
    # Wait 5 seconds
    time.sleep(3)
    global started
    # Reboot xserver if it crashed
    if (started == False):
        os.system("sudo systemctl restart xserver.service")

# Main function to start subscriber but also set display correctly


def init_video():
    print("STARTING")
    # Set the proper OS variable to display on
    os.environ['DISPLAY'] = ":0"
    # Start thread to do reboot
    bootThread = Thread(target=reboot, args=())
    bootThread.start()
    # Resets the display to resize correctly
    subprocess.run(["xrandr", "-o", "right"])
    subprocess.run(['xset', 'dpms', 'force', 'off'])
    global started
    started = True
    subprocess.run(['ledZero'])
