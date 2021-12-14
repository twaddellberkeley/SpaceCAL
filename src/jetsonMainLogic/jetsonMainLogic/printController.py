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
from interfaces.msg import PrintData
from dataclasses import dataclass
import queue
from threading import Thread

#Subscribe: What to print, where to print it, how fast to rotate, how long to print

#Publish: Where to go to print, how fast to rotate the motor, what to print




class printQueueClass(Node):

    @dataclass
    class printDataClass:
        speed: int
        length: int
        name: str
        location: int
        printernum: int
    
    printQ = queue.Queue()

    cSpeed = 0
    cLoc = 0
    cStatus = 0

    def __init__(self):
        super().__init__('queueClass')
        #publishers
        self.videoSend = self.create_publisher(String, 'videoName', 10)
        self.videoSend

        self.motorLocPublisher = self.create_publisher(Int32, 'setPosition', 10)
        self.motorVelPublisher = self.create_publisher(Int32, 'setVelocity', 10)
        self.motorLocPublisher
        self.motorVelPublisher
        #subscribers
        self.printSubscriber = self.create_subscription(
            PrintData, 'printSend', self.printAddQueue, 10)
        self.printSubscriber
                
        self.motorCLSubscriber = self.create_subscription(Int32, 'getPosition', self.getLocation, 10)
        self.motorCVSubscriber = self.create_subscription(Int32, 'getVelocity', self.getSpeed, 10)
        self.motorStatusSubscriber = self.create_subscription(Int32, 'getStatus', self.getStatus, 10)

        self.motorCLSubscriber
        self.motorCVSubscriber
        self.motorStatusSubscriber
        
        qThread = Thread(target=self.qPrint, args=())
        qThread.start()
        #self.qPrint()
    
    def getSpeed(self, msg):
        self.cSpeed = msg.data
    def getLocation(self, msg):
        self.cLoc = msg.data
    def getStatus(self,msg):
        self.cStatus = msg.data

    def printAddQueue(self, msg):
        print("got info")
        self.printQ.put(self.printDataClass(
            msg.speed,
            msg.length,
            msg.name,
            msg.location,
            msg.printernum
        ))

    def qPrint(self):
        while rclpy.ok():
            if(not self.printQ.empty()):
                printArg = self.printQ.get()
                print("Printing: " + str(printArg.name))
                #publish speed
                intMsg = Int32()
                intMsg.data = printArg.speed
                self.motorVelPublisher.publish(intMsg)                
                #publish location & wait for return
                intMsg.data = printArg.location
                self.motorLocPublisher.publish(intMsg)
                #wait for motor to be in position
                while(self.cLoc != printArg.location):
                    pass
                #publish name to print
                self.videoSend.publish(printArg.name)
                time.sleep(printArg.length)

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
