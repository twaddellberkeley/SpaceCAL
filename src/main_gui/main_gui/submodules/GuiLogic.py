# This Python file uses the following encoding: utf-8
import threading
import os
import rclpy
from rclpy.node import Node
import time
from threading import Thread
from functools import partial
from queue import Queue

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

gui_inpu_srv = 'gui_input_srv'
gui_display_srv = 'gui_display_srv'


class GuiLogic(QtWidgets.QWidget):

    updateState = pyqtSignal(int)
    updateHomePageDisplay = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        # this is a fuction that takes a command and sends the re
        # request to the server from the client

        self.isPrinting = False
        # we shoud recive a message when level is moving and ends moving: zero meas no moving
        self.isMovingCount = 0
        self.subNode = Node("main_gui_client_node")
        self.logger = self.subNode.get_logger()
        self.cli = self.subNode.create_client(GuiInput, gui_inpu_srv)

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
            if req.display_name == "level-status":
                if req.status == STATE_MOVING:
                    self.isMovingCount += 1
                    self.updateGuiState(req.state)
                elif req.status == STATE_STOPPED:
                    self.isMovingCount -= 1
                    self.updateGuiState(STATE_READY)
                if self.isMovingCount == 0 and self.isPrinting:
                    # send signal to start video
                    self.client_req("pi-play-queue-all")
                    t = threading.Timer(40, self.client_req, args=[
                                        "level-motors-next"])
                    t.daemon = True
                    t.start()
                    print("System should not be moving")
            else:
                self.updateGuiState(req.state)

    ################################# End Server Service ###################################

    ################################# Client Service ###################################

    @pyqtSlot(str)
    def decodeCmd(self, cmd):
        # this cmd comes from the gui
        # Metadata meta
        # int32 id
        # string cmd
        # string[] update_queue
        if "stop" in cmd:
            self.isPrinting = False
        elif "start" in cmd:
            self.isPrinting = True

        # This send command to server
        cliThread = threading.Thread(target=self.client_req, args=[cmd])
        cliThread.daemon = True
        cliThread.start()
        # self.client_req(cmd)
        print(cmd)

    def client_req(self, cmd):
        req = GuiInput.Request()
        req.id = 10
        req.cmd = cmd

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('service not available, waiting again...')

        self.logger.warning("[MainWindow]: sending client cmd %s" % (req.cmd))
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


# def serverData(self, req):

#         if "display" in req.cmd:
#             p = str(req.printer_id)
#             if req.printer_id < 0:
#                 p = "all"
#             self.gui.guiLogic.updateGuiDisplay(
#                 req.display_name + "-" + req.display_msg + "-" + p)
#         if "state" in req.cmd:
#             if req.display_name == "level-status":
#                 if req.status == STATE_MOVING:
#                     self.isMovingCount += 1
#                 elif req.status == STATE_STOPPED:
#                     self.isMovingCount -= 1
#                 if self.isMovingCount == 0 and self.isPrinting:
#                     self.client_req("pi-play-queue-all")
#                     self.timeThread = threading.Timer(
#                         40, self.client_req, args=["level-motors-next"])
#                     self.timeThread.start()
#             self.gui.guiLogic.updateGuiState(req.state)
#         # if "videos" in req.cmd:
#         #     self.gui.guiLogic.setVideos(req.printer_id, req.pi_videos)
#         # if "queue" in req.cmd:
#         #     self.gui.guiLogic
#         pass
