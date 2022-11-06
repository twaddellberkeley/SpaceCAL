# This Python file uses the following encoding: utf-8
# PyQt5.QtWidgets
import sys


from PyQt5.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
                          QMetaObject, QObject, QPoint, QRect,
                          QSize, QTime, QUrl, Qt, pyqtSlot, pyqtSignal)

from PyQt5.QtWidgets import (QApplication, QFrame, QHBoxLayout, QLabel,
                             QLayout, QMainWindow, QPushButton, QSizePolicy,
                             QStatusBar, QTabWidget, QVBoxLayout, QWidget)

from .Components.HomePageWidget import HomePageWidget
from .Components.PrinterPageWidget import PrinterPageWidget
from .Components.Msgs import Msgs
from .GuiLogic import GuiLogic

STATE_STOPPED = 0x00
STATE_READY = 0x01
STATE_MOVING = 0x10
STATE_PRINTING = 0x100


class MainWindow(QMainWindow):
    stateSignal = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.resize(1024, 600)
        self.guiLogic = GuiLogic()
        self.sysState = STATE_STOPPED

        ############################## Widgets #############################
        self.tabWidget = QTabWidget()
        self.homePage = HomePageWidget()
        self.printerPage = PrinterPageWidget()

        # Editing Widgits
        self.tabWidget.setDocumentMode(True)

        ############################# Layouts ##############################
        self.mainLayout = QVBoxLayout(self.tabWidget)

        # Inserting widget tabs
        self.tabWidget.insertTab(0, self.homePage, "Home")
        self.tabWidget.insertTab(1, self.printerPage, "Printer Controller")

        # Setting the central widget for the main window to be the tabWidget
        self.setCentralWidget(self.tabWidget)

        ###################### pyqtSignals and pyqtSlots ############################
        self.homePage.cmdSignal.connect(self.sendCmd)
        self.printerPage.cmdSignal.connect(self.sendCmd)

        ###################### System state signals ############################
        self.guiLogic.updateState.connect(self.setSysState)
        self.guiLogic.updateState.connect(self.homePage.setHomePageState)
        self.guiLogic.updateState.connect(self.printerPage.setPrinterPageState)

        ########################### System state signals ##################################
        self.guiLogic.updateHomePageDisplay.connect(self.homePage.setDisplay)

        ########################### Started Run signals ##################################
        self.homePage.mainBtnsWidget.startedRunSig.connect(
            self.guiLogic.setIsPrinting)

        ##########################################################
        Msgs().initSystemMsg()
        Msgs().warningMsg("Motors Will Be Home")
        self.sendCmd("init-system")
        ##########################################################
        # Define the size policy for the tab widgit -- we want the minimum to be 700 by 500
        # sizePolicy = QSizePolicy(
        #     QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        # sizePolicy.setHorizontalStretch(0)
        # sizePolicy.setVerticalStretch(0)
#        self.setSizePolicy(sizePolicy)

    def sendCmd(self, cmd):
        # This fuction receives signals from homePage and printerPage and sends them to the logic
        self.guiLogic.decodeCmd(cmd)
#        print(cmd)

    @pyqtSlot(int)
    def setSysState(self, state):
        # This fuction takes a signal from the logic and sets the state of the system
        self.sysState = state
        self.stateSignal.emit(state)


# if __name__ == "__main__":
#     app = QApplication([])
#     window = MainWindow()
#     window.show()
#     sys.exit(app.exec())
