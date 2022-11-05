# This Python file uses the following encoding: utf-8
from readline import get_current_history_length
from traceback import print_stack
from .LevelWidget import LevelWidget
from PyQt5 import QtWidgets
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton)
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QListView, QFormLayout, QMessageBox,
                             QLabel, QHBoxLayout, QBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)

from .Msgs import Msgs
from .PrinterWidget import PrinterWidget
from .SelectBtnWidget import SelectBtnWidget
from .TwoBtnWidget import TwoBtnWidget
from .VideoWidget import VideoWidget


STATE_STOPPED = 0x00
STATE_READY = 0x01
STATE_MOVING = 0x10
STATE_PRINTING = 0x100


class PrinterPageWidget(QtWidgets.QWidget):
    cmdSignal = pyqtSignal(str)
    stateSignal = pyqtSignal(int)

    stopBtnMsg = "Would you like stop the system?"
    startBtnMsg = "Would you like start printing?"

    stopMainBtnCmd = "stop-system"
    startMainBtnCmd = "stop-system"

    def __init__(self):
        super().__init__()
        # dummy structurs
        self.state = STATE_STOPPED

        ############################## Widgets #############################
        self.printerWidget = PrinterWidget()
        self.videoWidget = VideoWidget(self.printerWidget.getCurrPrinter())
        self.levelWidget = LevelWidget()
        self.mainBtns = TwoBtnWidget(u"Stop", u"Print")

        ######################### Message object ##########################
        self.msgs = Msgs()
        self.msgBox = QMessageBox()
        self.msgBox.setStandardButtons(QMessageBox.No | QMessageBox.Yes)
        self.msgBox.setDefaultButton(QMessageBox.Yes)

        ############################# Layouts ##############################
        self.hLayouts = [QHBoxLayout] * 4
        self.verticalLayout = QVBoxLayout(self)
        for i in range(4):
            self.hLayouts[i] = QHBoxLayout()

        ####################### Add widgets to Layouts #######################
        self.hLayouts[0].insertWidget(0, self.printerWidget.powerWidget)
        self.hLayouts[0].insertWidget(
            1, self.printerWidget.selectPrinterWidget)
        self.hLayouts[1].insertWidget(0, self.printerWidget.ledWidget)
        self.hLayouts[1].insertWidget(1, self.levelWidget.levelSelect)
        self.hLayouts[2].insertWidget(0, self.videoWidget.videoWidget)
        self.hLayouts[2].insertWidget(1, self.printerWidget.motorWidget)
        self.hLayouts[3].insertWidget(0, self.mainBtns)

        for i in range(4):
            self.verticalLayout.insertLayout(i, self.hLayouts[i])

        ###################### pyqtSignals and pyqtSlots ############################
        # Printer
        self.printerWidget.btnPressed.connect(self.sendCmdSignal)
        self.printerWidget.printerChanged.connect(
            self.videoWidget.updatePrinter)

        # Video
        self.videoWidget.btnPressed.connect(self.sendCmdSignal)

        # Main Buttons
        self.mainBtns.stopBtn.clicked.connect(self.stopMainBtn)
        self.mainBtns.startBtn.clicked.connect(self.constructPrintCmd)

    def sendCmdSignal(self, sig):
        self.cmdSignal.emit(sig)

    def stopMainBtn(self):
        self.msgBox.setText(self.stopBtnMsg)
        ret = self.msgBox.exec()
        if ret == QMessageBox.Yes:
            self.sendCmdSignal(self.stopMainBtnCmd)

    def startMainBtn(self):
        self.msgBox.setText(self.startBtnMsg)
        ret = self.msgBox.exec()
        if ret == QMessageBox.Yes:
            self.sendCmdSignal(self.constructPrintCmd)

    def constructPrintCmd(self):
        video = self.getCurrVideo()
        level = self.getCurrLevel()
        printer = self.getCurrPrinter()
        motorSpeed = self.getCurrMotorSpeed()

        if video == self.videoWidget.noVideo:
            return self.msgs.selecVideoMsg()
        if motorSpeed == "0":
            return self.msgs.selectSpeedMsg()
        cmd = "proj-on-" + printer + "+" + \
            "level-motors-" + level + "+" + \
            "motor-on-" + motorSpeed + "-" + printer + "+" + \
            "proj-led-on-" + printer + "+" + \
            "pi-play-" + video + "-" + printer
        ret = self.msgs.startPrintMsg()
        if ret == QMessageBox.Yes:
            self.sendCmdSignal(cmd)

    def getCurrVideo(self):
        return self.videoWidget.getCurrVideo()

    def getCurrLevel(self):
        return self.levelWidget.getCurrLevel()

    def getCurrPrinter(self):
        return self.printerWidget.getCurrPrinter()

    def getCurrMotorSpeed(self):
        return self.printerWidget.getCurrMotorSpeed()

    @pyqtSlot(int)
    def setPrinterPageState(self, state):
        # This fuction takes a signal from the logic and sets the state of the system
        self.state = state
        self.stateSignal.emit(state)
    # def addPrinterCmd(self, cmd):
    #     printer = self.printerWidget.getCurrPrinter()
    #     vCmd = cmd + printer
    #     self.sendCmdSignal(vCmd)
