# This Python file uses the following encoding: utf-8
from sys import displayhook
from PyQt5 import QtCore
from PyQt5 import QtWidgets

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QListView, QFormLayout,
                               QLabel, QHBoxLayout, QBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)


from Components.DisplayField import DisplayField


class SelectBtnWidget(QtWidgets.QWidget):
    def __init__(self, label="label", field="field", btnName="BtnName"):
        super().__init__()

        # widget
        self.display = DisplayField(label, field)
        self.btnSelect = QPushButton(btnName)

        # Layouts
        self.mainLayout = QHBoxLayout(self)

        self.mainLayout.insertWidget(0, self.display, 2)
        self.mainLayout.insertWidget(1, self.btnSelect, 1)
