# This Python file uses the following encoding: utf-8
from .Msgs import Msgs
from PyQt5 import QtCore
from PyQt5.QtCore import QSize, pyqtSlot, pyqtSignal
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QMessageBox,
                             QLabel, QHBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)

STATE_STOPPED = 0x00
STATE_READY = 0x01
STATE_MOVING = 0x10
STATE_PRINTING = 0x100


class MainBtnsWidget(QtWidgets.QWidget):
    btnPressed = pyqtSignal(str)
    startedRunSig = pyqtSignal(bool)
    startBtnRunText = u"Start Printing"
    stopBtnRunText = u"Stop Printing"
    startBtnText = u"Start Print"
    stopBtnText = u"End Print"

    def __init__(self):
        super().__init__()

        # Messages
        self.msgs = Msgs()
        self.mainBtnState = STATE_STOPPED
        # Widgets
        self.btnStop = QPushButton(u"Stop Printing")
        self.btnStart = QPushButton(u"Start Printing")

        # Set Widget properties
        self.btnStop.setProperty("StopBtn", True)
        self.btnStart.setProperty("StartBtn", True)
        self.btnStop.setMinimumSize(80, 120)
        self.btnStart.setMinimumSize(80, 120)
        # self.btnStop.setDisabled(True)        ########################## Uncomment

        # Layouts
        self.mainBtnsLayout = QHBoxLayout(self)

        # Add widgets to layouts
        self.mainBtnsLayout.addWidget(self.btnStop)
        self.mainBtnsLayout.addWidget(self.btnStart)
        # self.setStyleSheet(style)

        # Connect signals
        self.btnStart.clicked.connect(self.startBtnClicked)
        self.btnStop.clicked.connect(self.stopBtnClicked)

        # Set initial state of buttons
        self.setBtnsState(STATE_STOPPED)

    @pyqtSlot(bool)
    def setAutoPrintMode(self, mode):
        if mode == True:
            self.btnStop.setText(self.stopBtnRunText)
            self.btnStart.setText(self.startBtnRunText)
        else:
            self.btnStop.setText(self.stopBtnText)
            self.btnStart.setText(self.startBtnText)

    # This needs to be connected to a signal that emits when the state of the
    # printer changes
    @pyqtSlot(int)
    def setBtnsState(self, state):
        # Not printing
        # there are 5 componets to the systems: Projector on/off, led on/off,
        # svial rot on/off, video playing on/off and plataform moving on/off
        # We will represent this in binary: (proj, led, vial, video, plataform)
        # when one is on it gets a 1 and 0 otherwise. system fully on is (1,1,1,1,1)
        #        STE_STOPPED = 0x00
        #        STATE_READY = 0x01
        #        STATE_MOVING = 0x10
        #        STATE_PRINTING = 0x100

        if state == STATE_STOPPED:
            # Ready to print
            self.btnStart.setEnabled(True)
            self.btnStop.setDisabled(True)
            self.btnStart.setText(self.startBtnRunText)
            self.btnStop.setText(self.stopBtnRunText)
        # Printing
        elif state == STATE_READY:
            self.btnStart.setEnabled(True)
            self.btnStop.setEnabled(True)
            self.btnStart.setText(self.startBtnText)
            self.btnStop.setText(self.stopBtnText)
        elif state == STATE_MOVING:
            self.btnStart.setDisabled(True)
            self.btnStop.setDisabled(True)
            self.btnStart.setText(self.startBtnText)
            self.btnStop.setText(self.stopBtnText)
        elif state == STATE_PRINTING:
            self.btnStart.setDisabled(True)
            self.btnStop.setEnabled(True)
            self.btnStart.setText(self.startBtnText)
            self.btnStop.setText(self.stopBtnText)

     ##### pyqtSignal ######
    def startBtnClicked(self):
        ret = self.confirmStartBtnPress()
        if ret == QMessageBox.Yes:
            self.btnPressed.emit(self.getStartCmd())

     ##### pyqtSignal ######
    def stopBtnClicked(self):
        ret = self.confirmStopBtnPress()
        if ret == QMessageBox.Yes:
            self.btnPressed.emit(self.getStopCmd())

    def confirmStartBtnPress(self):
        ret = QMessageBox.No
        if self.btnStart.text() == self.startBtnRunText:
            ret = self.msgs.startPrintingMsg()
        elif self.btnStart.text() == self.startBtnText:
            ret = self.msgs.startPrintMsg()
        return ret

    def confirmStopBtnPress(self):
        ret = QMessageBox.No
        if self.btnStop.text() == self.stopBtnRunText:
            ret = self.msgs.stopPrintingMsg()
        elif self.btnStop.text() == self.stopBtnText:
            ret = self.msgs.stopPrintMsg()
        return ret

    def getStartCmd(self):
        if self.btnStart.text() == self.startBtnRunText:
            self.startedRunSig.emit(True)
            return self.msgs.start_run_cmd
        elif self.btnStart.text() == self.startBtnText:
            return self.msgs.start_print_cmd

    def getStopCmd(self):
        if self.btnStop.text() == self.stopBtnRunText:
            return self.msgs.stop_run_cmd
        elif self.btnStop.text() == self.stopBtnText:
            return self.msgs.stop_print_cmd


##### Style Sheets ######
style = """
QPushButton {
border: 2px solid #8f8f91;
border-radius: 6px;
background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                  stop: 0 #f6f7fa, stop: 1 #dadbde);
min-width: 80px;
min-height: 80px;
}


QPushButton:pressed {
background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                  stop: 0 #dadbde, stop: 1 #f6f7fa);
}

QPushButton:flat {
border: none; /* no border for a flat push button */
}

QPushButton:default {
border-color: navy; /* make the default button prominent */
}


"""

# QPushButton[isActive="false"] {
# background-color: gray;
# }
