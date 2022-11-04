# This Python file uses the following encoding: utf-8
# PyQt5.QtWidgets
import time
from datetime import datetime
from threading import Thread
import threading
from functools import partial
from queue import Queue
import sys


from PyQt5.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
                          QMetaObject, QObject, QPoint, QRect,
                          QSize, QTime, QUrl, Qt, pyqtSlot, pyqtSignal)

from PyQt5.QtWidgets import (QApplication, QFrame, QHBoxLayout, QLabel,
                             QLayout, QMainWindow, QPushButton, QSizePolicy,
                             QStatusBar, QTabWidget, QVBoxLayout, QWidget)

from .Components.HomePageWidget import HomePageWidget
from .Components.PrinterPageWidget import PrinterPageWidget
from interfaces.srv import GuiDisplay, GuiInput, Projector, Video, MotorSrv


class MainWindow(QMainWindow):
    currentState = 0
    stateSig = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.resize(1024, 600)

        self._client = None
        self._logger = None

        ############################## Widgets #############################
        self.tabWidget = QTabWidget()
        self.homePage = HomePageWidget()
        self.printPage = PrinterPageWidget()

        # Editing Widgits
        self.tabWidget.setDocumentMode(True)

        ############################# Layouts ##############################
        self.mainLayout = QVBoxLayout(self.tabWidget)

        # Inserting widget tabs
        self.tabWidget.insertTab(0, self.homePage, "Home")
        self.tabWidget.insertTab(1, self.printPage, "Printer Controller")

        # Setting the central widget for the main window to be the tabWidget
        self.setCentralWidget(self.tabWidget)

        ###################### pyqtSignals and pyqtSlots ############################
        self.homePage.cmdpyqtSignal.connect(self.sendCmd)
        self.printPage.cmdpyqtSignal.connect(self.sendCmd)

        # Define the size policy for the tab widgit -- we want the minimum to be 700 by 500
        # sizePolicy = QSizePolicy(
        #     QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        # sizePolicy.setHorizontalStretch(0)
        # sizePolicy.setVerticalStretch(0)
#        self.setSizePolicy(sizePolicy)

    def sendCmd(self, cmd):
        # Metadata meta
        # int32 id
        # string cmd
        # string[] update_queue

        req = GuiInput.Request()
        req.id = 10
        req.cmd = cmd

        while not self._client.wait_for_service(timeout_sec=1.0):
            self._logger.info('service not available, waiting again...')

        self._logger.warning("[MainWindow]: sending client cmd %s" % (req.cmd))
        future = self._client.call_async(req)
        # Add callback to receive response
        future.add_done_callback(partial(self.response_callback))
        print(cmd)

    def response_callback(self, future):
        # Metadata meta
        # int32 id
        # string cmd
        # int32 err
        # string msg
        try:
            res = future.result()

        except Exception as e:
            self._logger.error('ERROR: --- %r' % (e,))

    @pyqtSlot(int)
    def setSysState(self, state):
        assert type(state) == type(0), "Wrong type: must be an int"
        assert state < 32, "state must be less than 32"
        self.currentState |= state


# if __name__ == "__main__":
#     app = QApplication([])
#     window = MainWindow()
#     window.show()
#     sys.exit(app.exec())
