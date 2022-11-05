# This Python file uses the following encoding: utf-8
from PyQt5 import QtCore
from PyQt5 import QtWidgets

from PyQt5.QtCore import QObject, Qt
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QListView, QFormLayout,
                             QLabel, QHBoxLayout, QBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)


class DisplayField(QtWidgets.QWidget):

    def __init__(self, l="label", f="field"):
        super().__init__()

        # Widgets
        self.label = QLabel(l+":")
        self.field = QLabel(f)

        self.label.setProperty(u"objectName", u"label")
        self.field.setProperty(u"objectName", u"field")

        # Frame
        self.fieldFrame = QFrame()

        # Set Frame properties
        self.fieldFrame.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)

        self.setSizePolicy(
            QSizePolicy.Preferred, QSizePolicy.Fixed)

        # Layouts
        self.widgetLayout = QHBoxLayout(self)
        self.fieldFrameLayout = QHBoxLayout(self.fieldFrame)

        # Add Widgets to layouts
        self.fieldFrameLayout.insertWidget(0, self.field)
        self.widgetLayout.insertWidget(0, self.label, 1)
        self.widgetLayout.insertWidget(1, self.field, 2)
        # self.widgetLayout.addRow(self.label, self.field)

        # Widget properties
        self.field.setFrameStyle(2)
        self.field.setAlignment(Qt.AlignCenter)
        self.label.setAlignment(
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight)
        self.field.setMinimumHeight(20)
        self.field.setMargin(10)


#        self.setStyleSheet(style)

    def changeValue(self, value):
        self.field.setText(value)


style = """

QLabel#field {
border: 2px solid #8f8f91;
border-radius: 6px;
background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                  stop: 0 white, stop: 1 gray);
min-width: 10px;
min-height: 30px;
}

"""

# QLabel#label {
# text-align: right;
# border-radius: 6px;
# background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                  stop: 0 #f6f7fa, stop: 1 #dadbde);
# min-width: 10px;
# min-height: 30px;
# }
