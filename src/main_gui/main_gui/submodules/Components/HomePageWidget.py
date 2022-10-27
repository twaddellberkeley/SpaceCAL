# This Python file uses the following encoding: utf-8
from Components.DisplayWidget import DisplayWidget
from Components.PrinterModeWidget import PrinterModeWidget
from Components.MainBtnsWidget import MainBtnsWidget
from PySide6 import QtCore
from PySide6 import QtWidgets
from PySide6.QtCore import QSize, Slot, Signal
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame,
                               QLabel, QHBoxLayout, QSizePolicy)


class HomePageWidget(QtWidgets.QWidget):
    cmdSignal = Signal(str)

    def __init__(self):
        super().__init__()

        # Widgets
        self.printerModeFrame = QFrame()
        self.displayFrame = QFrame()
        self.mainBtnsFrame = QFrame()
        self.mainBtnsWidget = MainBtnsWidget()
        self.printerModeWidget = PrinterModeWidget()
        self.displayWidget = DisplayWidget()

        # Set Widgets sizePolicy This sets the ration betweeng the three frames
        sizePolicy = QFrame().sizePolicy()
        # The top fram is the smallest of the 3 frames
        sizePolicy.setVerticalStretch(1)
        self.printerModeFrame.setSizePolicy(sizePolicy)
        # The middle fram is the largets as is the one displaying the data
        sizePolicy.setVerticalStretch(4)
        self.displayFrame.setSizePolicy(sizePolicy)
        # This fram has the button to control the printer
        sizePolicy.setVerticalStretch(2)
        self.mainBtnsFrame.setSizePolicy(sizePolicy)

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

        # Signals and slots
        # Changes the state of the buttons from auto printing to manual
        self.printerModeWidget.modeChanged.connect(
            self.mainBtnsWidget.setAutoPrintMode)

        # Send cmd signals
        # self.printerModeWidget.modeChanged.connect(self.sendCmdSignal)
        self.printerModeWidget.btnPressed.connect(self.sendCmdSignal)
        self.mainBtnsWidget.btnPressed.connect(self.sendCmdSignal)

    def sendCmdSignal(self, sig):
        self.cmdSignal.emit(sig)
        # print(sig)
