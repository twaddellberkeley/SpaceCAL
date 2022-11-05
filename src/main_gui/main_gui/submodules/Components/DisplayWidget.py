# This Python file uses the following encoding: utf-8

from PyQt5 import QtWidgets

# QStringListModel, QStringList
from PyQt5.QtCore import QStringListModel, pyqtSignal, pyqtSlot

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QListView, QFormLayout,
                             QLabel, QHBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)

from .DisplayField import DisplayField


class DisplayWidget(QtWidgets.QWidget):
    printerData = ["Printer #", "Current Video",
                   "Status", "Vial rot/min"]
    printerDefaultState = ["#", "No Video", "Off", "0"]

    printerViews = [QListView] * 6
    printerModel = [QStringListModel] * 6

    def __init__(self):
        super().__init__()

#        # Frames
#        self.printerModeFrame = QFrame()
#        self.displayFrame = QFrame()
#        self.mainBtnsFrame = QFrame()

#        # Set Widgets sizePolicy This sets the ration betweeng the three frames
#        sizePolicy = QFrame().sizePolicy()
#        # The top frame is the smallest of the 3 frames
#        sizePolicy.setVerticalStretch(1)
#        self.printerModeFrame.setSizePolicy(sizePolicy)
#        # The middle fram is the largets as is the one displaying the data
#        sizePolicy.setVerticalStretch(4)
#        self.displayFrame.setSizePolicy(sizePolicy)
#        # This fram has the button to control the printer
#        sizePolicy.setVerticalStretch(2)
#        self.mainBtnsFrame.setSizePolicy(sizePolicy)

        # Widgets
        self.parabolaDisplayWidget = DisplayField("Parabola", "0")
        self.gravityDisplayWidget = DisplayField("Gravity", "9.8")
        self.levelDisplayWidget = DisplayField("Current Level", "Home")
        self.printerModel[0] = QStringListModel(self.printerData)
        self.printerViews[0] = QListView()
        self.printerViews[0].setModel(self.printerModel[0])

        # Widget Properties

        for i in range(1, 6):
            self.printerModel[i] = QStringListModel(self.printerDefaultState)
            self.printerViews[i] = QListView()
            self.printerViews[i].setModel(self.printerModel[i])

        # Layouts
        self.displayLayout = QVBoxLayout(self)
        self.topDisplayLayout = QHBoxLayout()
        self.bottomDisplayLayout = QHBoxLayout()

        # Add Widgets to layouts
        for i in range(6):
            self.topDisplayLayout.insertWidget(i, self.printerViews[i])

        self.bottomDisplayLayout.insertWidget(0, self.levelDisplayWidget, 1)
        self.bottomDisplayLayout.insertWidget(1, self.parabolaDisplayWidget, 1)
        self.bottomDisplayLayout.insertWidget(2, self.gravityDisplayWidget, 1)
        self.displayLayout.insertLayout(0, self.topDisplayLayout, 3)
        self.displayLayout.insertLayout(1, self.bottomDisplayLayout, 1)

    @pyqtSlot(str)
    def setDisplay(self, cmd):
        # format of cmd: "displayName+displayField"

        print(cmd)
        self.parabolaDisplayWidget.changeValue(cmd)
