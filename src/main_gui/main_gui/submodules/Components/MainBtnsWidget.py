# This Python file uses the following encoding: utf-8
from Components.Msgs import Msgs
from PyQt5 import QtCore
from PyQt5.QtCore import QSize, pyqtSlot, pyqtSignal
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QMessageBox,
                               QLabel, QHBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)


class MainBtnsWidget(QtWidgets.QWidget):
    btnPressed = pyqtSignal(str)
    startBtnAutoText = u"Start Printing"
    stopBtnAutoText = u"Stop Printing"
    startBtnText = u"Start Print"
    stopBtnText = u"End Print"
    readyState = 0
    printingState = 1

    def __init__(self):
        super().__init__()

        # Messages
        self.msgs = Msgs()

        # Widgets
        self.btnStop = QPushButton("Stop Printing")
        self.btnStart = QPushButton("Start Printing")

        # Set Widget properties
        self.btnStop.setProperty("StopBtn", True)
        self.btnStart.setProperty("StartBtn", True)
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

    @pyqtSlot(bool)
    def setAutoPrintMode(self, mode):
        if mode == True:
            self.btnStop.setText(self.stopBtnAutoText)
            self.btnStart.setText(self.startBtnAutoText)
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
        print(state, int('0b00011', 2))
        if state == self.readyState:
            # Ready to print
            self.btnStart.setEnabled(True)
            self.btnStop.setDisabled(True)
        # Printing
        elif state == self.printingState:
            self.btnStart.setDisabled(True)
            self.btnStop.setEnabled(True)

     ##### pyqtSignal ######
    def startBtnClicked(self):
        # msgBox = QMessageBox()
        ret = QMessageBox.No
        if self.startBtnText == self.btnStart.text():
            ret = self.msgs.startPrintMsg()
            # msgBox.setText("Would you like begin print?")
        else:
            ret = self.msgs.startPrintingMsg()
            # msgBox.setText("Would you like begin auto printing?")
        # msgBox.setStandardButtons(QMessageBox.No | QMessageBox.Yes)
        # msgBox.setDefaultButton(QMessageBox.Yes)
        # ret = msgBox.exec()
        if ret == QMessageBox.Yes:
            self.btnPressed.emit(self.btnStart.text())

     ##### pyqtSignal ######
    def stopBtnClicked(self):
        ret = QMessageBox.No
        # msgBox = QMessageBox()
        if self.stopBtnText == self.btnStop.text():
            ret = self.msgs.endPrintMsg()
            # msgBox.setText("Would you like stop the print?")
        else:
            ret = self.msgs.stopSysMsg()
        #     msgBox.setText("Would you like to stop printing?")
        # msgBox.setStandardButtons(QMessageBox.No | QMessageBox.Yes)
        # msgBox.setDefaultButton(QMessageBox.Yes)
        # ret = msgBox.exec()
        if ret == QMessageBox.Yes:
            self.btnPressed.emit(self.btnStop.text())


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
