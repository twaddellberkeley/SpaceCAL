# This Python file uses the following encoding: utf-8
from PyQt5 import QtCore
from PyQt5 import QtWidgets

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QFormLayout, QMessageBox, QInputDialog,
                             QLabel, QHBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)


class Msgs(QtWidgets.QWidget):

    def __init__(self, parent=None):
        super().__init__()

        if parent == None:
            parent = self

        self.start_run_cmd = "start-run"
        self.start_print_cmd = "start-print"
        self.stop_run_cmd = "stop-run"
        self.stop_print_cmd = "stop-print"

        self.parent = parent
        self.initMsg = "SpaceCal"
        self.stopSys = "Would you like stop the system?"
        self.endPrint = "Would you like stop current print?"
        self.startPrinting = "Would you like to begin printing?"
        self.startPrint = "Would you like to begin print?"
        self.stopPrinting = "Would you like to stop the system?"
        self.stopPrint = "Would you like stop current print??"
        self.selectVideo = "Please Select a video"
        self.selectSpeed = "Please select a speed greater than zero"
        self.msgBox = QMessageBox()
        self.msgBox.setStandardButtons(QMessageBox.No | QMessageBox.Yes)
        self.msgBox.setDefaultButton(QMessageBox.Yes)
        self.msgInfo = QMessageBox()
        self.msgOptions = QInputDialog()
        self.initSystem = QMessageBox()
        self.msgWarning = QMessageBox()

    def initSystemMsg(self):
        # setButtonText(0, "initialize Systme")
        self.initSystem.setFixedSize(1024, 600)
        self.initSystem.setText(self.initMsg)
        return self.initSystem.exec()

    def warningMsg(self, msg):
        assert type("") == type(msg), "wrong type"
        self.msgWarning.setText(msg)
        self.msgWarning.setIcon(QMessageBox.Icon.Warning)
        return self.msgWarning.exec()

    def confirmMsg(self, msg):
        # print(type(""), type(msg))
        assert type("") == type(msg), "wrong type"
        self.msgBox.setText(msg)
        return self.msgBox.exec()

    def startPrintMsg(self):
        self.msgBox.setText(self.startPrint)
        return self.msgBox.exec()

    def startPrintingMsg(self):
        self.msgBox.setText(self.startPrinting)
        return self.msgBox.exec()

    def stopPrintMsg(self):
        self.msgBox.setText(self.stopPrint)
        return self.msgBox.exec()

    def stopPrintingMsg(self):
        self.msgBox.setText(self.stopPrinting)
        return self.msgBox.exec()

    def stopSysMsg(self):
        self.msgBox.setText(self.stopSys)
        return self.msgBox.exec()

    def endPrintMsg(self):
        self.msgBox.setText(self.endPrint)
        return self.msgBox.exec()

    def selecVideoMsg(self):
        self.msgInfo.setText(self.selectVideo)
        return self.msgInfo.exec()

    def selectSpeedMsg(self):
        self.msgInfo.setText(self.selectSpeed)
        return self.msgInfo.exec()

    def selectOptionVideoMsg(self, items):
        return self.msgOptions.getItem(
            self.parent, "Select a video", "Video:", items, 0, False)

    def selectOptionLevelMsg(self, items):
        return self.msgOptions.getItem(
            self, "Select a level", "Level:", items, 0, False)

    def selectOptionSpeedMsg(self, items):
        return self.msgOptions.getItem(
            self, "Select a speed", "Speed:", items, 0, False)

    def selectOptionPrinterMsg(self, items):
        return self.msgOptions.getItem(
            self, "Select a printer", "Printer:", items, 0, False)
