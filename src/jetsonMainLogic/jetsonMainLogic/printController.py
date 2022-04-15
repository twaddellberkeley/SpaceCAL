# Copyright 2021 UC-Berkeley
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Node that will recognize prints in the queue, and print accordingly
# Author: Taylor Waddell
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Int64
import time
from interfaces.msg import MotorData, DisplayData
from dataclasses import dataclass
import queue
from threading import Thread
from threading import Event
import json
import os

# Subscribe: What to print, where to print it, how fast to rotate, how long to print

# Publish: Where to go to print, how fast to rotate the motor, what to print


class printQueueClass(Node):

    # Data Class to send to pi to print
    @dataclass
    class printDataClass:
        speed: int
        length: int
        name: str
        printernum: int

    # Data class to send where to move
    @dataclass
    class baseMove:
        printNum: int
        maxTime: int
        printHeight: int
        printdata: list

    # Print queue to move base
    printQ = queue.Queue()
    # Var that allows print process to start
    okToRun = False
    #If resuming need to skip
    skipPop = False
    # Stop run and home
    killRun = False

    # Global Checkup Vars
    cSpeed = [-1] * 9
    cLoc = [-1] * 9
    cStatus = [-1] * 9
    cFlags = [0] * 9

    # Motor address offset
    mOffset = 14
    # Lift motors: 14,15,16,17
    # Rotation motors: 18,19,20,21,22

    # Converstion bits
    # incrBit = 51200 # Is one rotation or one inch
    incrBit = 1280  # Is 1mm per bit unit

    # Step veloicty to achieve 1rot/min
    velBit = 1024000000/60  # Max 500000000

    # Decoded JSON print set
    printSet = None

    # Thread event
    tEvent = Event()

    def __init__(self):
        super().__init__('queueClass')
        # publishers

        # Motors for moving base
        self.motorLocPublisher = self.create_publisher(
            Int32, 'setPosition', 10)
        self.motorLocPublisher

        # publishers for sending velecotiy
        self.motorVelPublisherV1 = self.create_publisher(
            Int32, 'setVelocityV1', 10)
        self.motorVelPublisherV1

        self.motorVelPublisherV2 = self.create_publisher(
            Int32, 'setVelocityV2', 10)
        self.motorVelPublisherV2

        self.motorVelPublisherV3 = self.create_publisher(
            Int32, 'setVelocityV3', 10)
        self.motorVelPublisherV3

        self.motorVelPublisherV4 = self.create_publisher(
            Int32, 'setVelocityV4', 10)
        self.motorVelPublisherV4

        self.motorVelPublisherV5 = self.create_publisher(
            Int32, 'setVelocityV5', 10)
        self.motorVelPublisherV5

        self.velocityPublishers = [self.motorVelPublisherV1, self.motorVelPublisherV2,
                                   self.motorVelPublisherV3, self.motorVelPublisherV4, self.motorVelPublisherV5]

        # publishers for sending video
        self.videoSendV1 = self.create_publisher(String, 'videoNameV1', 10)
        self.videoSendV1

        self.videoSendV2 = self.create_publisher(String, 'videoNameV2', 10)
        self.videoSendV2

        self.videoSendV3 = self.create_publisher(String, 'videoNameV3', 10)
        self.videoSendV3

        self.videoSendV4 = self.create_publisher(String, 'videoNameV4', 10)
        self.videoSendV4

        self.videoSendV5 = self.create_publisher(String, 'videoNameV5', 10)
        self.videoSendV5

        self.videoPublishers = [self.videoSendV1, self.videoSendV2,
                                self.videoSendV3, self.videoSendV4, self.videoSendV5]
        self.touchScreenPublisher = self.create_publisher(DisplayData, 'display_topic', 10)
        self.touchScreenPublisher
        # subscribers
        # Motor data subscriber
        self.motorDataSubscriber = self.create_subscription(
            MotorData, 'motorData', self.getMotorData, 10)
        # Keyboard input subscriber
        self.touchInputSubscriber = self.create_subscription(
            String,
            'buttons_topic',
            self.touchScreenHandler,
            10)
        self.touchInputSubscriber
        # Thread to send base change too
        # Read the print file
        self.readPrintFile()
        self.okToRun = False
        self.startProjection = False
        self.pauseAll = False
        qThread = Thread(target=self.qPrint, args=())
        # qThread.daemon=True
        qThread.start()

    # Get variables from everything
    # Get custom message from the motor, index info in its array, its
    # motornum is its address
    def getMotorData(self, msg):
        self.cSpeed[msg.motornum - self.mOffset] = msg.speed
        self.cStatus[msg.motornum - self.mOffset] = msg.status
        self.cLoc[msg.motornum - self.mOffset] = msg.location
        self.cFlags[msg.motornum - self.mOffset] = msg.flags

    def readPrintFile(self):
        script_dir = os.path.dirname(__file__)
        file = open(script_dir + "/printTest.json")
        # file = open("/home/spacecal/Desktop/printTest.json")
        data = json.load(file)
        # Loop through JSON and create object for each print set
        for val in data:
            if val['printNum'] != -1:
                printData = self.baseMove(
                    val['printNum'],
                    val['maxTime'],
                    val['printHeight'],
                    val['prints']
                )
                self.printQ.put(printData)
            else:
                # If -1 then we must home
                self.printQ.put(-1)

    # Subscribe to touchscreen variables
    def touchScreenHandler(self, msg):
        if (msg.data == "stop_proj"):
            self.killProjection()
        elif (msg.data == "start_proj"):
            self.startProjection = True
        elif (msg.data == "motor_ok"):
            self.okToRun = True
        elif (msg.data == "pause"):
            self.tEvent.set()
            self.okToRun = False
            self.pauseAll = True
        elif (msg.data == "options"):
            self.options = True
        elif (msg.data == "resume"):
            print("resuming")
            self.okToRun = True
            self.tEvent.clear()
            self.skipPop = True
        elif (msg.data == "kill"):
            self.okToRun = False
            self.tEvent.clear()
            self.killRun = True

    # Loop for printing EVERYTHING
    def qPrint(self):
        global printSet
        # Value for message later
        vel = Int32()
        # Location to go to publish
        loc = Int32()
        # Data to send to touch screen
        tdata = DisplayData()
        # Setting OG values
        tdata.name = "motor_status"
        tdata.str_value = "Idle"
        self.touchScreenPublisher.publish(tdata)
        tdata.name = "level_status"
        tdata.str_value = "Idle"
        self.touchScreenPublisher.publish(tdata)
        # Keep running while alive
        while rclpy.ok():
            # Keep running until there is not a print to go, only run if it is safe
            while ((not self.printQ.empty() or self.skipPop == True) and self.okToRun == True):
                # Get the current print job in queue
                print("Running set")
                if (self.skipPop == False):
                    printSet = self.printQ.get()
                self.skipPop = False
                # Wait for motors to be good and online
                for val in range(len(self.cStatus)):
                    while(self.cStatus[val] != 10):
                        print("waiting for motors to come online")
                        time.sleep(.1)
                        pass
                # Go Home
                if(printSet == -1):
                    # Go home, do this by sending -1 to distance
                    loc.data = printSet
                    print("Homing")
                    tdata.name = "level_status"
                    tdata.str_value = "Homing"
                    self.touchScreenPublisher.publish(tdata)
                    self.motorLocPublisher.publish(loc)
                    # Wait for home to complete from all mototrs, need this line as it sometimes jumps ahead
                    time.sleep(1)
                    for val in range(4):
                        print(str(val) + "Flag num " + str(self.cFlags[val]))
                        while ((self.cFlags[val] & 0x10) == 0x10 or (self.cFlags[val] & 0x02) == 0x02):
                            time.sleep(.1)
                            if(self.okToRun == False):
                                break
                            pass
                        # If postion is uncertain than something bad has happened
                        if ((self.cFlags[val] & 0x02) == 1):
                            print("ERROR IN HOMING")
                        if(self.okToRun == False):
                                break
                    print("Done Homing")
                    tdata.name = "level_status"
                    tdata.str_value = "Homed"
                    self.touchScreenPublisher.publish(tdata)
                # Begin printing normal printer data
                else:
                    if (printSet.printNum > 0):
                        print("Starting print number: " +
                              str(printSet.printNum))
                    else:
                        print("Returning to 0")
                    # Max height we can go
                    if (printSet.printHeight > 310):
                        printSet.printHeight = 310
                    loc.data = printSet.printHeight
                    print("running to height" + str(printSet.printHeight))
                    self.motorLocPublisher.publish(loc)

                    # Send level in mm
                    tdata.name = "level_display"
                    tdata.num_value = printSet.printHeight
                    self.touchScreenPublisher.publish(tdata)
                    tdata.name = "level_status"
                    tdata.str_value = "Moving"
                    self.touchScreenPublisher.publish(tdata)
                    # Wait till all motors are at correct height before starting projections
                    for val in range(4):
                        while(round(self.cLoc[val]) != round(printSet.printHeight * self.incrBit)):
                            if(self.okToRun == False):
                                break
                            pass
                        if(self.okToRun == False):
                                break
                    tdata.name = "level_status"
                    tdata.str_value = "Set"
                    self.touchScreenPublisher.publish(tdata)
                    if (printSet.printNum > 0):
                        #Set current parabola we are on
                        tdata.name = "parabola_display"
                        tdata.num_value = printSet.printNum
                        self.touchScreenPublisher.publish(tdata)
                        # Loop through and and thread to prep print each print on current set
                        for val in printSet.printdata:
                            qThread = Thread(
                                target=self.readyPart, args=([val]))
                            qThread.daemon = True
                            qThread.start()
                        tdata.name = "motor_status"
                        tdata.str_value = "Rotating"
                        self.touchScreenPublisher.publish(tdata)
                        # Set up each video, but do not display
                        # Wait for user input to display
                        while (not self.startProjection):
                            if(self.okToRun == False):
                                break
                            pass
                        if(self.okToRun == False):
                            break
                        # Unset variable for next time
                        self.startProjection = False
                        # Loop through and turn projectors on
                        for val in printSet.printdata:
                            # print(val)
                            qThread = Thread(
                                target=self.projectorOn, args=([val]))
                            qThread.daemon = True
                            qThread.start()
                        # Wait the max display time before movingW
                        while not self.tEvent.is_set():
                            self.tEvent.wait(timeout=printSet.maxTime)
                            self.tEvent.set()
                        # Kill all projections just in case
                        self.killProjection()
                        # Unset the event for next time
                        self.tEvent.clear()
                        if(self.okToRun == False):
                            break
                    # If printSet is 0, we have moved back to 0/home, need to turn off rotation
                    if(printSet.printNum == 0):
                        self.okToRun = False
                        tdata.name = "reset_run"
                        self.touchScreenPublisher.publish(tdata)
                        # Set all rotation to 0
                        for velPublisher in self.velocityPublishers:
                            vel.data = 0
                            velPublisher.publish(vel)
                        tdata.name = "rpm_display"
                        tdata.num_value = 0
                        self.touchScreenPublisher.publish(tdata)
                        tdata.name = "motor_status"
                        tdata.str_value = "Stopped"
                        self.touchScreenPublisher.publish(tdata)
                        print("Waiting for next vial stack to be loaded")
            if (self.pauseAll == True or self.killRun == True):
                # Stop the repeating of the function
                self.okToRun = False
                # Stop all rotation
                for velPublisher in self.velocityPublishers:
                    vel.data = 0
                    velPublisher.publish(vel)
                # Stop all projections
                self.killProjection()
                # Tell motors to stop where current location is
                #loc.data = self.cLoc[0]
                print("pausing")
                if (self.pauseAll == True):
                    loc.data = -2
                if (self.killRun == True):
                    loc.data = -1
                self.motorLocPublisher.publish(loc)
                self.tEvent.set()
                tdata.name = "rpm_display"
                tdata.num_value = 0
                self.touchScreenPublisher.publish(tdata)
                tdata.name = "motor_status"
                tdata.str_value = "Stopped"
                self.touchScreenPublisher.publish(tdata)
                self.killRun = False
                self.pauseAll = False

    # Set all projector LEDs to 0

    def killProjection(self):
        tdata = DisplayData()
        tdata.name = "reset_projection"
        self.touchScreenPublisher.publish(tdata)
        print("stoping projection")
        video = String()
        video.data = "EXIT"
        for val in self.videoPublishers:
            val.publish(video)

    # Send video to project and get up to speed

    def readyPart(self, printData):
        print("Printing: " + str(printData['videoName']) +
              " on projector" + str(printData['projNum']))
        # Set publish types
        vel = Int32()
        video = String()
        tdata = DisplayData()
        # Get data from array
        vel.data = printData['printSpeed']
        video.data = printData['videoName']
        tdata.name = "rpm_display"
        tdata.num_value = printData['printSpeed']
        self.touchScreenPublisher.publish(tdata)
        # Send to projector and motor
        self.velocityPublishers[printData['projNum'] - 1].publish(vel)
        print(video.data)
        self.videoPublishers[printData['projNum'] - 1].publish(video)

    def projectorOn(self, printData):
        video = String()
        video.data = "PRINT"
        self.videoPublishers[printData['projNum'] - 1].publish(video)
        # Wait set time to print
        print("WAITING")
        time.sleep(printData['printTime'])
        # kill projection just in case
        print("KILLING")
        video.data = "EXIT"
        # Kill Projection
        self.videoPublishers[printData['projNum'] - 1].publish(video)


def main(args=None):
    rclpy.init(args=args)
    publisher = printQueueClass()
    rclpy.spin(publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
