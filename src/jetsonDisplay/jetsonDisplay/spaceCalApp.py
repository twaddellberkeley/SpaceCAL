from re import L
from tkinter import Button
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QMainWindow, QHBoxLayout, QMessageBox, QVBoxLayout
from PyQt5 import uic, QtTest
import sys


class UI(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("spaceCalApp.ui", self)

        # Buttons: (QPushButton)
        #   OptionBtn
        #   ProjectorStartBtn
        #   ProjectorStopBtn
        #   StartRunBtn
        #   PausePrintingBtn

        # Labels: (QLabel)
        #   statusOutput
        #   typeOutput
        #   R_statusOutput
        #   L_statusOutput

        # Labels: (QLCDNumber)
        #   parabolaCountOutput
        #   accelVectorOutput
        #   R_rpmOutput
        #   L_levelOutput


class UIW(QWidget):
    def __init__(self):
        super().__init__()
        uic.loadUi("form.ui", self)


app = QApplication(sys.argv)
window = UI()
window2 = UIW()
window.show()
window2.show()
app.exec()
