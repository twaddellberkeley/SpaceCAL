# This Python file uses the following encoding: utf-8
from PyQt5 import QtCore
from PyQt5 import QtWidgets

from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QListView, QFormLayout, QMessageBox,
                               QLabel, QHBoxLayout, QBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)


class TwoBtnWidget(QtWidgets.QWidget):
    btnPressed = pyqtSignal(str)

    def __init__(self, stop="Off", start="On"):
        super().__init__()
        # Widgets
        self.stopBtn = QPushButton(stop)
        self.startBtn = QPushButton(start)
        # self.widget = QWidget()

        # Layouts
        self.mainLayout = QHBoxLayout(self)

        # Add Widgets
        self.mainLayout.insertWidget(0, self.stopBtn)
        self.mainLayout.insertWidget(1, self.startBtn)

        # pyqtSignals and pyqtSlots
        self.stopBtn.clicked.connect(self.stopBtnPressed)
        self.startBtn.clicked.connect(self.startBtnPressed)

    def stopBtnPressed(self):
        self.btnPressed.emit(self.stopBtn.text())

    def startBtnPressed(self):
        self.btnPressed.emit(self.startBtn.text())
