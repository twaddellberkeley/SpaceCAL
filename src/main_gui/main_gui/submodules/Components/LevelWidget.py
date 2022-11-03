# This Python file uses the following encoding: utf-8
from Components.Msgs import Msgs
from PyQt5 import QtCore
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QInputDialog,
                               QLabel, QHBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)
from Components.SelectBtnWidget import SelectBtnWidget


class LevelWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        # Messages
        self.msgs = Msgs()

        # Widget
        self.levelSelect = SelectBtnWidget(u"Level", u"5", u"Select...")

        # pyqtSignals and pyqtSlots
        self.levelSelect.btnSelect.clicked.connect(self.levelSelectBtnPressed)

    def levelSelectBtnPressed(self):
        items = ["Home", "1", "2", "3", "4", "5"]
        # item, ok = QInputDialog.getItem(
        #     self, "Select desired level", "level", items, 0, False)
        item, ok = self.msgs.selectOptionLevelMsg(items)
        if ok and item:
            self.levelSelect.display.field.setText(item)

    def getCurrLevel(self):
        return self.levelSelect.display.field.text().lower()
