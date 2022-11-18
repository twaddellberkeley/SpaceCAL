# This Python file uses the following encoding: utf-8
from .DisplayWidget import DisplayWidget
from .PrinterModeWidget import PrinterModeWidget
from .MainBtnsWidget import MainBtnsWidget
from .Msgs import Msgs
from PyQt5 import QtCore
from PyQt5 import QtWidgets
from PyQt5.QtCore import QSize, pyqtSlot, pyqtSignal
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame,
                             QLabel, QHBoxLayout, QSizePolicy)

STATE_STOPPED = 0x00
STATE_READY = 0x01
STATE_MOVING = 0x10
STATE_PRINTING = 0x100


class HomePageWidget(QtWidgets.QWidget):
    cmdSignal = pyqtSignal(str)
    stateSignal = pyqtSignal(int)
    displaySignal = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self.start_run_cmd = "start-run"
        self.state = 0x00

        # Frames
        self.printerModeFrame = QFrame()
        self.displayFrame = QFrame()
        self.mainBtnsFrame = QFrame()

        # Set Widgets sizePolicy This sets the ration betweeng the three frames
        sizePolicy = QFrame().sizePolicy()
        # The top frame is the smallest of the 3 frames
        sizePolicy.setVerticalStretch(1)
        self.printerModeFrame.setSizePolicy(sizePolicy)
        # The middle fram is the largets as is the one displaying the data
        sizePolicy.setVerticalStretch(4)
        self.displayFrame.setSizePolicy(sizePolicy)
        # This fram has the button to control the printer
        sizePolicy.setVerticalStretch(2)
        self.mainBtnsFrame.setSizePolicy(sizePolicy)

        # Widgets
        self.mainBtnsWidget = MainBtnsWidget()
        self.printerModeWidget = PrinterModeWidget()
        self.displayWidget = DisplayWidget()

        # Set Widget properties
        self.printerModeFrame.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        self.displayFrame.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        self.mainBtnsFrame.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)

        # Layouts
        self.homePageLayout = QVBoxLayout(self)
        self.mainBtnsFrameLayout = QVBoxLayout(self.mainBtnsFrame)
        self.printerModeFrameLayout = QVBoxLayout(self.printerModeFrame)
        self.displayFrameLayout = QVBoxLayout(self.displayFrame)

        # Add widgets to Layouts
        self.printerModeFrameLayout.addWidget(self.printerModeWidget)
        self.displayFrameLayout.addWidget(self.displayWidget)
        self.mainBtnsFrameLayout.addWidget(self.mainBtnsWidget)
        self.homePageLayout.addWidget(self.printerModeFrame)
        self.homePageLayout.addWidget(self.displayFrame)
        self.homePageLayout.addWidget(self.mainBtnsFrame)

        # pyqtSignals and slots
        # Changes the state of the buttons from auto printing to manual
        self.printerModeWidget.modeChanged.connect(
            self.mainBtnsWidget.setAutoPrintMode)

        # Send cmd signals
        # self.printerModeWidget.modeChanged.connect(self.sendCmdSignal)
        self.printerModeWidget.btnPressed.connect(self.sendCmdSignal)
        self.mainBtnsWidget.btnPressed.connect(self.sendCmdSignal)
        self.stateSignal.connect(self.printerModeWidget.setBtnsState)
        self.stateSignal.connect(self.mainBtnsWidget.setBtnsState)
        self.displaySignal.connect(self.displayWidget.setDisplay)

    def sendCmdSignal(self, sig):
        self.cmdSignal.emit(sig)
        # print(sig)

    @pyqtSlot(int)
    def setHomePageState(self, state):
        self.state = state
        self.stateSignal.emit(state)

    @pyqtSlot(str)
    def setDisplay(self, cmd):
        # cmd format "displayName+displayField"
        self.displaySignal.emit(cmd)