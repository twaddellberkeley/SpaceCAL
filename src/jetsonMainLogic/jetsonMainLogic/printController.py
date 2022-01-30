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

#Node that will recognize prints in the queue, and print accordingly
# Author: Taylor Waddell
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Int64
import time
from interfaces.msg import MotorData
from dataclasses import dataclass
import queue
from threading import Thread
import json
import os

#Subscribe: What to print, where to print it, how fast to rotate, how long to print

#Publish: Where to go to print, how fast to rotate the motor, what to print




class printQueueClass(Node):

    #Data Class to send to pi to print
    @dataclass
    class printDataClass:
        speed: int
        length: int
        name: str
        printernum: int
    
    #Data class to send where to move
    @dataclass 
    class baseMove:
        printNum: int
        maxTime: int
        printHeight: int
        printdata: list

    #Print queue to move base 
    printQ = queue.Queue()
    #Var that allows print process to start
    okToRun = False

    #Global Checkup Vars
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
    incrBit = 51200/25.4 # Is 1mm per bit unit

    # Step veloicty to achieve 1rot/min
    velBit = 1024000000/60 # Max 500000000

    def __init__(self):
        super().__init__('queueClass')
        #publishers

        #Motors for moving base
        self.motorLocPublisher = self.create_publisher(Int32, 'setPosition', 10)
        self.motorLocPublisher
        
        #publishers for sending velecotiy
        self.motorVelPublisherV1 = self.create_publisher(Int32, 'setVelocityV1', 10)
        self.motorVelPublisherV1

        self.motorVelPublisherV2 = self.create_publisher(Int32, 'setVelocityV2', 10)
        self.motorVelPublisherV2

        self.motorVelPublisherV3 = self.create_publisher(Int32, 'setVelocityV3', 10)
        self.motorVelPublisherV3

        self.motorVelPublisherV4 = self.create_publisher(Int32, 'setVelocityV4', 10)
        self.motorVelPublisherV4

        self.motorVelPublisherV5 = self.create_publisher(Int32, 'setVelocityV5', 10)
        self.motorVelPublisherV5

        self.velocityPublishers = [self.motorVelPublisherV1,self.motorVelPublisherV2,self.motorVelPublisherV3,self.motorVelPublisherV4,self.motorVelPublisherV5]

        #publishers for sending video
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

        self.videoPublishers = [self.videoSendV1,self.videoSendV2,self.videoSendV3,self.videoSendV4,self.videoSendV5]

        # subscribers
        # Motor data subscriber
        self.motorDataSubscriber = self.create_subscription(MotorData, 'motorData', self.getMotorData, 10)
        # Keyboard input subscriber
        self.KeyboardInputSubscriber = self.create_subscription(
            String,
            'keyinput',
            self.letsLaunch,
            10)
        # TODO add keyboard input to simulate touch screen
        # Thread to send base change too
        qThread = Thread(target=self.qPrint, args=())
        qThread.daemon=True
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
        # TODO create solid path for this
        #script_dir = os.path.dirname(__file__)
        #file = open(script_dir + "/printTest.json")
        file = open("/home/spacecal/Desktop/printTest.json")
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
                #If -1 then we must home
                self.printQ.put(-1)

    def letsLaunch(self,msg):
        if (msg.data =="g"):
            self.okToRun = True

    def qPrint(self):
        #Read the print file
        self.readPrintFile()
        #Value for message later
        vel = Int32()
        while rclpy.ok():
            if(not self.printQ.empty() and self.okToRun == True):
                printSet = self.printQ.get()
                print(self.printQ.qsize())
                loc = Int32()
                # Wait for motors to be good
                for val in range(len(self.cStatus) - 1):
                    while(self.cStatus[val] != 10):
                        print("waiting for motors to come online")
                        time.sleep(.1)
                        pass
                # Go Home
                if(printSet == -1):
                    #Go home, do this by sending -1 to distance
                    loc.data = printSet
                    print("running")
                    self.motorLocPublisher.publish(loc)
                    #Wait for home to complete from all mototrs
                    for val in range(3):
                        while ((self.cFlags[val] & 0x10) == 0x10 or (self.cFlags[val] & 0x02) == 0x02):
                            print(self.cFlags[val])
                            time.sleep(.1)
                            pass
                        # If postion is uncertain than something bad has happened
                        if ((self.cFlags[val] & 0x02) == 1):
                            print("ERROR IN HOMING")
                    print("Done Homing")
                # Begin printing normal printer data
                else:
                    if (printSet.printNum > 0):
                        print("Starting print number: " + str(printSet.printNum))
                    else:
                        print("Returning to 0")
                    loc.data = printSet.printHeight
                    print("running")
                    self.motorLocPublisher.publish(loc)
                    #Wait till all motors are at correct height before starting projections
                    for val in range(3):
                        while(self.cLoc[val]* self.incrBit != printSet.printHeight):
                            print(self.cLoc[val]*self.incrBit)
                            pass
                    #Loop through and and thread to print each print on current set
                    for val in printSet.printdata:
                        #print(val)
                        qThread = Thread(target=self.printPart, args=([val]))
                        qThread.daemon=True
                        qThread.start()
                    time.sleep(printSet.maxTime)
                    # If printSet is 0, we have moved back to 0/home
                    if(printSet.printNum == 0):
                        self.okToRun = False
                        #Set all rotation to 0
                        for velPublisher in self.velocityPublishers:
                            vel.data = 0
                            velPublisher.publish(vel)
                        print("Waiting for next vial stack to be loaded")

    
    def printPart(self, printData):
        print("Printing: " + str(printData['videoName']) + " on projector" + str(printData['projNum']))
        # Set publish types
        vel = Int32()
        video = String()
        # Get data from array
        vel.data = printData['printSpeed']
        video.data = printData['videoName']
        # Send to projector and motor
        self.velocityPublishers[printData['projNum'] - 1].publish(vel)
        self.videoPublishers[printData['projNum'] - 1].publish(video)
        # Wait set time to print
        time.sleep(printData['printTime'])
        # kill projection just in case
        video.data = "EXIT"
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
