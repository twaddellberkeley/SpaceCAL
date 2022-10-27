# This Python file uses the following encoding: utf-8
from sys import displayhook
from PySide6 import QtCore
from PySide6 import QtWidgets

from PySide6.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QListView, QFormLayout,
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
