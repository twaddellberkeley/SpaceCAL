# This Python file uses the following encoding: utf-8
from distutils.sysconfig import get_config_var
from Components.Msgs import Msgs
from PySide6 import QtCore
from PySide6 import QtWidgets

from PySide6.QtCore import QObject, Signal, Slot

from PySide6.QtWidgets import (QWidget, QVBoxLayout, QPushButton, QFrame, QListView, QFormLayout, QInputDialog,
                               QLabel, QHBoxLayout, QBoxLayout, QSizePolicy, QStyleOptionButton, QStyle)
from Components.SelectBtnWidget import SelectBtnWidget
from Components.TwoBtnWidget import TwoBtnWidget


class VideoWidget(QtWidgets.QWidget):
    btnPressed = Signal(str)

    noVideo = u"No_Video_Selected"

    def __init__(self, printer):
        super().__init__()

        ######### Creating Dummy videos ############
        self.currPrinter = printer
        self.videos = [list] * 5
        for i in range(len(self.videos)):
            lst = []
            for indx in range(10):
                lst.insert(indx, "hello/video/" + str(indx))
            self.videos[i] = lst

        # print(self.videos)

        # Messages
        self.msgs = Msgs(self)
        # Widgets
        self.videoWidget = QWidget()
        self.videoLabel = SelectBtnWidget(
            u"Video", self.noVideo, u"Select...")
        self.videoBtns = TwoBtnWidget(u"Stop", u"Play")

        # Layouts
        self.videoLayout = QVBoxLayout(self.videoWidget)

        # Add Widgets to the Layouts
        self.videoLayout.insertWidget(0, self.videoLabel)
        self.videoLayout.insertWidget(1, self.videoBtns)

        # Signals and Slots
        self.videoLabel.btnSelect.clicked.connect(self.videoSelectBtnPressed)
        self.videoBtns.startBtn.clicked.connect(self.videoBtnsStartPressed)
        self.videoBtns.stopBtn.clicked.connect(self.videoBtnsStopPressed)

    def videoBtnsStartPressed(self):
        video = self.getCurrVideo()
        printer = self.getCurrPrinter()
        if video == self.noVideo:
            return self.msgs.selecVideoMsg()
        self.btnPressed.emit("pi-play-" + video + "-" + printer)

    def videoBtnsStopPressed(self):
        printer = self.getCurrPrinter()
        self.btnPressed.emit("pi-stop-video-" + printer)

    def videoSelectBtnPressed(self):
        items = self.getVideos()
        item, ok = self.msgs.selectOptionVideoMsg(items)
        if ok and item:
            self.videoLabel.display.field.setText(item)

    def getCurrVideo(self):
        return self.videoLabel.display.field.text()

    @Slot(str)
    def updatePrinter(self, printer):
        assert type("") == type(printer), "Wrong type"
        printer = printer.lower()
        if printer != self.currPrinter:
            self.currPrinter = printer.lower()
            self.videoLabel.display.field.setText(self.noVideo)

    def getVideos(self):
        printer = self.getCurrPrinter()
        assert type(printer) == type(""), "Wrong type"
        if printer == "all":
            length = len(self.videos[0])
            return [str(i) for i in range(length)]
        return self.videos[int(printer)-1]

    def getCurrPrinter(self):
        return self.currPrinter
