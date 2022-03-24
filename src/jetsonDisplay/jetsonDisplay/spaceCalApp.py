from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QMainWindow, QHBoxLayout, QMessageBox
from PyQt5 import uic, QtTest
import sys


class UI(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("spaceCalMainWindow.ui", self)

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

        # CSS button styles
        self.enGreenBtnCss = "background-color:#1BC118; color:white;"
        self.enRedBtnCss = "background-color:#FF0000; color:white;"
        self.disBtnCss = "background-color:gray; color:white;"

        # Callback functions for button clicks
        self.OptionBtn.clicked.connect(self.onClick_optionBtn)
        self.ProjectorStartBtn.clicked.connect(self.onClick_prjStartBtn)
        self.ProjectorStopBtn.clicked.connect(self.onClick_prjStopBtn)
        self.StartRunBtn.clicked.connect(self.onClick_startRunBtn)
        self.PausePrintingBtn.clicked.connect(self.onClick_pauseBtn)

        # Set Button states
        self.disBtn(self.ProjectorStartBtn)
        self.disBtn(self.ProjectorStopBtn)
        self.disBtn(self.PausePrintingBtn)
        self.StartRunBtn.setText("Initiate Test")
        self.StartRunBtn.setStyleSheet("background-color:yellow;")

        # message confirmation window
        self.msgWindow = QMessageBox()
        self.msgWindow.setIcon(QMessageBox.Question)
        self.msgWindow.setStandardButtons(QMessageBox.Yes | QMessageBox.Cancel)
        self.msgWindow.setDefaultButton(QMessageBox.Yes)
        self.popButtons = self.msgWindow.buttons()
        style = """QLabel{font-size: 30px; text-align:center;}
            QWidget icon{heigh: 60px}
            QPushButton{
                min-width:300px; 
                min-height:100px; 
                font-size: 30px;
                background-color:#CCCCCC; 
                color:#000000;
                border-radius:0.2em; 
                padding:0.5em 1.0em;
                margin:0 0.3em 0.3em 0;
                text-decoration:none;
                }"""
        self.msgWindow.setStyleSheet(style)
        self.msgWindow.defaultButton().setStyleSheet(
            "QPushButton{background-color:#3369ff; color:#FFFFFF;}")

    def onClick_optionBtn(self):
        self.statusOutput.setText("Option")

    def onClick_prjStartBtn(self):
        self.msgWindow.setIcon(QMessageBox.Question)
        self.msgWindow.setText("Start Projection?")
        ret = self.msgWindow.exec()
        if (QMessageBox.Yes == ret):
            print("Send Command to ROS: Start Projecton")
            self.statusOutput.setText("On")
            self.enRedBtn(self.ProjectorStopBtn)
            self.disBtn(self.ProjectorStartBtn)
            self.statusOutput.setStyleSheet("QLabel{color: green;}")

    def onClick_prjStopBtn(self):
        self.msgWindow.setIcon(QMessageBox.Warning)
        self.msgWindow.setText("Are you sure you want to stop the projection?")
        ret = self.msgWindow.exec()
        if (QMessageBox.Yes == ret):
            self.msgWindow.setText(
                "Please Confirm You want to stop projection")
            ret = self.msgWindow.exec()
            if (QMessageBox.Yes == ret):
                print("Send command to ROS: Stop Projection")
                self.enGreenBtn(self.ProjectorStartBtn)
                self.disBtn(self.ProjectorStopBtn)
                self.statusOutput.setText("Off")
                self.statusOutput.setStyleSheet("QLabel{color: red;}")

    def onClick_startRunBtn(self):
        self.msgWindow.setIcon(QMessageBox.Question)
        if (self.StartRunBtn.text() == "Start Run"):
            self.msgWindow.setText("Would you like to start Run?")
            ret = self.msgWindow.exec()
            if (QMessageBox.Yes == ret):
                print("Send Command to ROS: Start Run")
                print("Subscribe to ROS: get RPM value")
                self.enGreenBtn(self.ProjectorStartBtn)
                self.enRedBtn(self.PausePrintingBtn)
                self.enRedBtn(self.StartRunBtn)
                self.StartRunBtn.setText("Stop Run")
                self.setRotationStatus("On")
        elif (self.StartRunBtn.text() == "Stop Run"):
            self.msgWindow.setText("Would you like to stop Run?")
            ret = self.msgWindow.exec()
            if (QMessageBox.Yes == ret):
                self.msgWindow.setText("Confirm stop Run?")
                ret = self.msgWindow.exec()
                if (QMessageBox.Yes == ret):
                    print("send command to ROS: Stop Run")
                    self.disBtn(self.PausePrintingBtn)
                    self.enGreenBtn(self.StartRunBtn)
                    self.StartRunBtn.setText("Start Run")
        else:
            self.msgWindow.setIcon(QMessageBox.Information)
            self.msgWindow.setText("Setting level to home!")
            ret = self.msgWindow.exec()
            print("Send command to ROS: Set Home Level")
            self.setLevelStatus("Reseting...")
            self.disBtn(self.StartRunBtn)
            QtTest.QTest.qWait(3000)
            self.enGreenBtn(self.StartRunBtn)
            self.StartRunBtn.setText("Start Run")
            self.setLevelStatus("Home")

    def onClick_pauseBtn(self):
        self.msgWindow.setText("Are you sure you want to pause the system?")
        ret = self.msgWindow.exec()
        if (QMessageBox.Yes == ret):
            popWin = QMessageBox()
            popWin.setIcon(QMessageBox.Information)
            popWin.setText("System has been Pause")
            popWin.setStandardButtons(QMessageBox.Yes)
            popWin.buttons()[0].setText("Resume")
            popWin.exec()
            self.statusOutput.setText("pause")

    def enRedBtn(self, btn):
        btn.setEnabled(True)
        btn.setStyleSheet(self.enRedBtnCss)

    def enGreenBtn(self, btn):
        btn.setEnabled(True)
        btn.setStyleSheet(self.enGreenBtnCss)

    def disBtn(self, btn):
        btn.setDisabled(True)
        btn.setStyleSheet(self.disBtnCss)

    def setLevelStatus(self, str):
        self.L_statusOutput.setText(str)
        self.L_statusOutput.setStyleSheet("color:green;")

    def setRotationStatus(self, str):
        self.R_statusOutput.setText(str)
        self.R_statusOutput.setStyleSheet("color:green;")

    def setRpm():
        pass

    def setLevel():
        pass

    def setParabolaCount(self):
        pass

    def setAccelVector(self):
        pass

    def setProjType(sef):
        pass

    def setProjStatus(self):
        pass


app = QApplication(sys.argv)
window = UI()
window.show()
app.exec()
