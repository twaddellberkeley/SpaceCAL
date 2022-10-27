# This Python file uses the following encoding: utf-8
from PySide6 import QtCore
from PySide6 import QtWidgets

from PySide6.QtCore import QObject, Signal, Slot
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QListView, QFormLayout, QMessageBox,
                               QLabel, QHBoxLayout, QBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)


class TwoBtnWidget(QtWidgets.QWidget):
    btnPressed = Signal(str)

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

        # Signals and Slots
        self.stopBtn.clicked.connect(self.stopBtnPressed)
        self.startBtn.clicked.connect(self.startBtnPressed)

    def stopBtnPressed(self):
        self.btnPressed.emit(self.stopBtn.text())

    def startBtnPressed(self):
        self.btnPressed.emit(self.startBtn.text())
