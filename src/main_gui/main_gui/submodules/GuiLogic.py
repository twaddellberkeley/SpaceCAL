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
    serverRequest = pyqtSignal(str)
    cli_req = None

    def __init__(self):
        super().__init__()
        # this is a fuction that takes a command and sends the re
        # request to the server from the client

        # self.logger = None
        self.subNode = Node("main_gui_client_node")
        self.logger = self.subNode.get_logger()
        self.cli = self.subNode.create_client(GuiInput, gui_inpu_srv)
        self.serverRequest.connect(self.decodeServerCmd)

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

    def decodeServerCmd(self, cmd):

        print(cmd)
        pass

    @pyqtSlot(str)
    def decodeCmd(self, cmd):
        # this cmd comes from the gui
        # Metadata meta
        # int32 id
        # string cmd
        # string[] update_queue

        # This send command to server
        cliThread = threading.Thread(target=self.client_req, args=[cmd])
        cliThread.daemon = True
        cliThread.start()
        # self.client_req(cmd)
        print(cmd)

    def server_req(self, cmd):
        # this comes from the server
        self.serverRequest.emit(cmd)

    def updateGuiState(self, state):
        self.updateState.emit(state)

    def updateGuiDisplay(self, display):
        # display formant: "displayName+displayField"
        self.updateHomePageDisplay.emit(display)
