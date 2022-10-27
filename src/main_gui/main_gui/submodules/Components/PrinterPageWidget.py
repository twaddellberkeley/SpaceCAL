# This Python file uses the following encoding: utf-8
from readline import get_current_history_length
from traceback import print_stack
from Components.LevelWidget import LevelWidget
from PySide6 import QtWidgets
from PySide6.QtCore import QObject, Signal, Slot
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QPushButton)
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QListView, QFormLayout, QMessageBox,
                               QLabel, QHBoxLayout, QBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)

from Components.Msgs import Msgs
from Components.PrinterWidget import PrinterWidget
from Components.SelectBtnWidget import SelectBtnWidget
from Components.TwoBtnWidget import TwoBtnWidget
from Components.VideoWidget import VideoWidget


class PrinterPageWidget(QtWidgets.QWidget):
    cmdSignal = Signal(str)

    stopBtnMsg = "Would you like stop the system?"
    startBtnMsg = "Would you like start printing?"

    stopMainBtnCmd = "stop-system"
    startMainBtnCmd = "stop-system"

    def __init__(self):
        super().__init__()
        # dummy structurs

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

        ###################### Signals and Slots ############################
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
        cmd = "proj-on-" + printer + "_" + \
            "level-motors-" + level + "_" + \
            "motor-on-" + motorSpeed + "-" + printer + "_" + \
            "proj-led-on-" + printer + "_" + \
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

    # def addPrinterCmd(self, cmd):
    #     printer = self.printerWidget.getCurrPrinter()
    #     vCmd = cmd + printer
    #     self.sendCmdSignal(vCmd)
