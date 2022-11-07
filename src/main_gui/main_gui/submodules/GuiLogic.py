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

JSON_FILE = "/home/spacecal/Desktop/flightManifest.json"

gui_inpu_srv = 'gui_input_srv'
gui_display_srv = 'gui_display_srv'



class GuiLogic(QtWidgets.QWidget):

    updateState = pyqtSignal(int)
    updateHomePageDisplay = pyqtSignal(str)
    is_done_moving = pyqtSignal(str)


    def __init__(self):
        super().__init__()
        # this is a fuction that takes a command and sends the re
        # request to the server from the client
        self.sysState = STATE_STOPPED
        self.current_level = 0
        self.parabola_number = 0
        self.keepRunning = False
        # we shoud recive a message when level is moving and ends moving: zero meas no moving
        self.isMovingCount = 0
        self.subNode = Node("main_gui_client_node")
        self.logger = self.subNode.get_logger()
        self.cli = self.subNode.create_client(GuiInput, gui_inpu_srv)

        self.printQ = Queue()

        self.readPrintFile()
        self.current_print = self.printQ.get()

        self.is_done_moving.connect(self.ready_next_print)



    def setIsPrinting(self, started):
        # TODO: set a button that will change the state of is printing in main butons
        if started:
            self.keepRunning = True
        else:
            self.keepRunning = False

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
                    
                elif req.state == STATE_STOPPED:
                    self.isMovingCount -= 1
                    self.logger.warning('*************isMovingCount is: %d\n' %(self.isMovingCount))
                    
                if self.isMovingCount == 0:
                    # Do what needs to be done at this level
                    self.is_done_moving.emit(req.display_msg)

        self.logger.warning('*************Display Name: %s\n' %(req.display_name))
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
        #     self.keepRunning = False
        # elif "start" in cmd:
        #     self.keepRunning = True
        send_cmd = ""
        if cmd == "init-system":
            self.client_req("level-motors-home")
            send_cmd = "proj-on-all+motor-off-all+proj-led-off-all+level-motor-0"
            self.updateGuiState(STATE_MOVING)
        elif cmd == "start-run":
            print(self.current_print)
            # Get level ready for prints
            self.setLevelPrints()

        elif cmd == "start-print":
            
            self.sendTimedPrint(self.current_print['prints'])
            self.updateGuiState(STATE_PRINTING)

            #send time to move to next level
            moveThread = threading.Thread(target=self.sendTimeToMove, args=[self.current_print['maxTime']])
            moveThread.daemon = True
            moveThread.start()
            
            
        elif cmd == "stop-print":
            send_cmd = "motor-off-all+proj-led-off-all"
            self.keepRunning = False
            self.updateGuiState(STATE_STOPPED)
        if send_cmd == "":
            print("No valid command given")
            # This send command to server
            # cliThread = threading.Thread(target=self.client_req, args=[send_cmd])
            # cliThread.daemon = True
            # cliThread.start()
        else:
            self.client_req(send_cmd)
        print(cmd)


    def ready_next_print(self, current_level):

        # This function is call everytime we go from one leve to the other
        self.logger.warning('System is Fully stopped: %d\n' %(self.isMovingCount))
        if not current_level.isnumeric():
            # if we are not at a specific level then there is nothing we need to do
            self.logger.warning('Level is not numeric: %s\n' %(current_level))
            return 
        print(current_level)
        # Udate the our current level and the parabola value
        self.current_level = int(current_level)
        self.parabola_number += 1

        # Check if the queue has more prints, if not reload
        if self.printQ.empty():
            self.readPrintFile()
            self.logger.info('Reloding queue cuz is empty level: %s\n' %(current_level))
        # Get the next printing item
        self.current_print = self.printQ.get()

        # Check if the system has been marked to stop or we need to reload the viles
        if (not self.keepRunning) or (self.current_level == 0):
            self.updateGuiState(STATE_STOPPED)
            self.stopSystem()
            # if we are here because the queue is empty
            self.logger.warning('System is not Running Or we are at level: %s\n' %(current_level))
        
        # If not then we need to load the next print
        else:
            # Load the next videos and move motors
            self.setLevelPrints()
            

    def sendVidoes(self, printList):
        if printList == None:
            # No videos 
            #set the sate to stopped a
            # set isPrinting to False
            #stop the system
            return
        for printer in printList:
            self.client_req("pi-play-" + printer["videoName"] + "-" + str(printer["projNum"] - 1))
        

    def sendMotorSpeeds(self, speedList):
        if speedList == None:
            # No videos 
            #set the sate to stopped a
            # set isPrinting to False
            #stop the system
            return
        for printer in speedList:
           self.client_req("motor-on-" + str(printer["printSpeed"]) + "-"+ str(printer["projNum"] - 1))
    

    def sendTimedPrint(self, printList):
        if printList == None:
            # No videos 
            #set the sate to stopped a
            # set isPrinting to False
            #stop the system
            return
        for printer in printList:
            self.client_req("proj-led-on-" + str(printer["projNum"] - 1),  printer['printTime'])
        

    def sendTimeToMove(self, maxTime):
        # set state for the button
        time.sleep(maxTime)
        self.client_req("level-motors-next")
    
    def setLevelPrints(self):
        self.logger.warning('**** Getting level: %s for print ****\n' %(self.current_level))
        # make sure leds are off
        self.client_req("proj-led-off-all")

        # play the videos
        self.sendVidoes(self.current_print['prints'])

        # start rotating the motors
        self.sendMotorSpeeds(self.current_print['prints'])

        # Change to button state
        self.updateGuiState(STATE_READY)

    
    def stopSystem(self):
        self.client_req("proj-led-off-all+motor-off-all+pi-stop-video-all")
        

    def client_req(self, cmd, t=None):
        req = GuiInput.Request()
        req.id = -1
        if t != None:
            req.id = t
            print(req.id)
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
        self.sysState = state

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
