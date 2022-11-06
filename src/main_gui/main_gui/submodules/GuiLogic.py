# This Python file uses the following encoding: utf-8
import threading
import os
import rclpy
from rclpy.node import Node
import time
from threading import Thread
from functools import partial
from queue import Queue
import json
import sys

from PyQt5 import QtCore
from PyQt5 import QtWidgets

from PyQt5.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
                          QMetaObject, QObject, QPoint, QRect,
                          QSize, QTime, QUrl, Qt, pyqtSlot, pyqtSignal)

from PyQt5.QtWidgets import (QApplication, QFrame, QHBoxLayout, QLabel,
                             QLayout, QMainWindow, QPushButton, QSizePolicy,
                             QStatusBar, QTabWidget, QVBoxLayout, QWidget)

from .Components.Msgs import Msgs
from interfaces.srv import GuiDisplay, GuiInput, Projector, Video, MotorSrv
import interfaces

STATE_STOPPED = 0x00
STATE_READY = 0x01
STATE_MOVING = 0x10
STATE_PRINTING = 0x100

JSON_FILE = "/home/spacecal/Desktop/printTest.json"

gui_inpu_srv = 'gui_input_srv'
gui_display_srv = 'gui_display_srv'



class GuiLogic(QtWidgets.QWidget):

    updateState = pyqtSignal(int)
    updateHomePageDisplay = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        # this is a fuction that takes a command and sends the re
        # request to the server from the client
        self.sysState = STATE_STOPPED
        self.current_level = 0
        self.parabola_number = 0
        self.isPrinting = False
        # we shoud recive a message when level is moving and ends moving: zero meas no moving
        self.isMovingCount = 0
        self.subNode = Node("main_gui_client_node")
        self.logger = self.subNode.get_logger()
        self.cli = self.subNode.create_client(GuiInput, gui_inpu_srv)

        self.printQ = Queue()

        self.readPrintFile()
        self.current_print = self.printQ.get()

    def setIsPrinting(self, started):
        # TODO: set a button that will change the state of is printing in main butons
        if started:
            self.isPrinting = True
        else:
            self.isPrinting = False

    ################################# Server Service #######################################

    def serverData(self, req):

        if "display" in req.cmd:
            p = str(req.printer_id)
            if req.printer_id < 0:
                p = "all"
            self.updateGuiDisplay(
                req.display_name + "-" + req.display_msg + "-" + p)
        if "state" in req.cmd:
            if req.display_name == "level-state":
                # self.logger.warning('*************isMovingCount is: %d\n' %(self.isMovingCount))
                if req.state == STATE_MOVING:
                    self.isMovingCount += 1
                    self.logger.warning('*************isMovingCount is: %d\n' %(self.isMovingCount))
                    self.updateGuiState(req.state)
                    self.sysState = req.state
                elif req.state == STATE_STOPPED:
                    self.isMovingCount -= 1
                    self.logger.warning('*************isMovingCount is: %d\n' %(self.isMovingCount))
                    
                if self.isMovingCount == 0:
                    if self.isPrinting:
                        self.updateGuiState(STATE_READY)
                        self.sysState = STATE_READY
                        self.logger.warning('"System is Ready to print": %d\n' %(self.isMovingCount))
                        # start the next video to play
                    else:
                        self.updateGuiState(STATE_STOPPED)
                        self.sysState = STATE_STOPPED
                        self.logger.warning('"System is Fully stopped": %d\n' %(self.isMovingCount))
                    self.logger.warning('*************isMovingCount is: %d\n' %(self.isMovingCount))
                if req.display_msg.isnumeric():
                    self.current_level == int(req.display_msg)
                    self.parabola_number += 1
                    # TODO: send a signal to the parabola display
                    ## Update current level
                    if self.current_level == 4:
                        # tell the command that the next level is to load the viles
                        pass

        self.logger.warning('*************Currrent SYSTEM STATE: %d\n' %(self.sysState))
                # self.updateGuiState(req.state)

    ################################# End Server Service ###################################

    ################################# Client Service ###################################

    @pyqtSlot(str)
    def decodeCmd(self, cmd):
        # this cmd comes from the gui
        # Metadata meta
        # int32 id
        # string cmd
        # string[] update_queue
        # if "stop" in cmd:
        #     self.isPrinting = False
        # elif "start" in cmd:
        #     self.isPrinting = True
        send_cmd = ""
        if cmd == "init-system":
            send_cmd = "level-motors-home+proj-on-all+motor-off-all+proj-led-off-all+level-motor-0"
        elif cmd == "start-run":

            # make sure leds are off
            self.client_req("proj-led-off")

            # play the videos
            self.sendVidoes(self.current_print['prints'])

            # start rotating the motors
            self.sendMotorSpeeds(self.current_print['prints'])

            # This always happens after we load new viles
            #send_cmd = "proj-led-off-all+level-motors-0+motor-on-9-all+pi-play-queue-all"

        elif cmd == "start-print":
            # Set the state to printing
            self.updateGuiState(STATE_PRINTING)
            printList = None
            # pop the first object from the queue
            if self.printQ.empty():
                self.updateGuiState(STATE_STOPPED)
                self.isPrinting = False

                # self.client_req("level-motor-0+motor-off-all+proj-led-off-all")

            #send projectors commands
            if self.current_level == 4:
                isPrinting = False
            
            self.sendTimedPrint(self.current_print['prints'])

            #send time to move to next level
            moveThread = threading.Thread(target=self.sendTimeToMove, args=[self.current_print['maxTime'], printList])
            
            # for fun in range(5):
            #     t = threading.Thread(target=self.printTime, args=[])
            # send led times and videos for each printerfrom the queue
            # self.client_req("proj-led-on-2", 10)
            
        elif cmd == "stop-print":
            send_cmd = "motor-off-all+proj-led-off-all"
            self.isPrinting = False
        if send_cmd == "":
            print("No valid command given")
            # This send command to server
            # cliThread = threading.Thread(target=self.client_req, args=[send_cmd])
            # cliThread.daemon = True
            # cliThread.start()
        else:
            self.client_req(send_cmd)
        print(cmd)



    def sendVidoes(self, printList):
        if printList == None:
            # No videos 
            #set the sate to stopped a
            # set isPrinting to False
            #stop the system
            return
        for printer in printList:
            self.client_req("pi-play-video-" + printer["videoName"] + "-" + str(printer["projNum"]))
        

    def sendMotorSpeeds(self, speedList):
        if speedList == None:
            # No videos 
            #set the sate to stopped a
            # set isPrinting to False
            #stop the system
            return
        for printer in speedList:
           self.client_req("motor-on-" + str(printer["printSpeed"]) + "-"+ str(printer["projNum"]))
    

    def sendTimedPrint(self, printList):
        if printList == None:
            # No videos 
            #set the sate to stopped a
            # set isPrinting to False
            #stop the system
            return
        for printer in printList:
            self.client_req("proj-led-on-" + str(printer["projNum"]),  printer['printTime'])
        pass

    def sendTimeToMove(self, maxTime, printList=None):
        # set state for the button
        time.sleep(int(maxTime))

        if printList == None:
            self.client_req("level-motors-next")
            self.stopSystem()
            return

        self.client_req("level-motors-next")
        time.sleep(1)
        self.sendVidoes(printList)
        self.sendMotorSpeeds(printList)
            

    
    def stopSystem(self):
        self.client_req("proj-led-off-all+motor-off-all+pi-stop-video-all")
        

    def client_req(self, cmd, t=None):
        req = GuiInput.Request()
        req.id = -1
        if t != None:
            print(req.id)
            req.id = t
        req.cmd = cmd
        time_to_wait = 4
        count = 0
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('service not available, waiting again...')
            # if count > time_to_wait:
            #     return
            # count += 1

        self.logger.warning("[MainWindow]: sending client cmd: ***** %s *****" % (req.cmd))
        future = self.cli.call_async(req)
        # Add callback to receive response
        future.add_done_callback(partial(self.response_callback))
        print(cmd)

    def response_callback(self, future):
        print("Finished Callback")
    ################################# End Client Service ###################################

    def updateGuiState(self, state):
        self.updateState.emit(state)

    def updateGuiDisplay(self, display):
        # display formant: "displayName+displayField"
        self.updateHomePageDisplay.emit(display)

    def led_off_timed(self, t):
        self.client_req("proj-led-on-2")
        time.sleep(t)
        self.client_req("proj-led-off-2")

    


    def readPrintFile(self):
        # script_dir = os.path.dirname(__file__)
        # file = open(script_dir + "/printTest.json")
        with open(JSON_FILE) as f:
            data = json.load(f)
        # file = open("/home/spacecal/Desktop/printTest.json")
        
        # Loop through JSON and create object for each print set
        for val in data:
            if val['printNum'] != -1 and val['printNum'] != 0:
                self.printQ.put(val)


        # for val in data:
        #     if val['printNum'] != -1:
        #         printData = self.baseMove(
        #             val['printNum'],
        #             val['maxTime'],
        #             val['printHeight'],
        #             val['prints']
        #         )
        #         self.printQ.put(printData)
        #     else:
        #         # If -1 then we must home
        #         self.printQ.put(-1)
