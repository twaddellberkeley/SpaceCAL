# This Python file uses the following encoding: utf-8

from PyQt5 import QtWidgets

from PyQt5.QtCore import QStringListModel  # QStringListModel, QStringList

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QListView, QFormLayout,
                               QLabel, QHBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)

from Components.DisplayField import DisplayField


class DisplayWidget(QtWidgets.QWidget):
    printerData = ["Printer #", "Current Video",
                   "Status", "Vial rot/min"]
    printerDefaultState = ["#", "No Video", "Off", "0"]

    printerViews = [QListView] * 6
    printerModel = [QStringListModel] * 6

    def __init__(self):
        super().__init__()

        # Widgets
        self.parabolaDisplayWidget = DisplayField("Parabola", "0")
        self.gravityDisplayWidget = DisplayField("Gravity", "9.8")
        self.levelDisplayWidget = DisplayField("Current Level", "Home")
        self.printerModel[0] = QStringListModel(self.printerData)
        self.printerViews[0] = QListView()
        self.printerViews[0].setModel(self.printerModel[0])

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
