from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import uic, QtTest

import time
import traceback
import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

# QMessageBox messages text for setText
initRunMsg = "Motors will be set to home position!"
startRunMsg = "Would you like to start motor functionality?"
stopRunMsg = "Would you like to stop motor functionality?"
confStopRunMsg = "STOPING MOTTOR FUNCTIONALITY?"
startProjectionMsg = "Would you like to start projecting?"
stopProjectionMsg = "Would you like to stop projecting?"
pauseAllMsg = "Would you like to pause all running proccesses?"
pauseInfoMsg = "Press 'OK' to resume all proccesses"

# Button string names:
runBtnInit = "Initialize Run"
runBtnStart = "Start Run"
runBtnStop = "Stop Run"
projectBtnStart = "Start Projection"
projectBtnStop = "Stop Projection"
optionBtn = "Options"
pauseBtnPause = "Pause"
pauseBtnResume = "Resume"

###### *********************** ROS2 Variables **************************** ######
# ROS2 Nodes
pubNodeStr = "buttons_node"
subNodeStr = "display_node"
# ROS2 Subscriber Topic names
statusProjectorStr = "projector_status"
statusMotorStr = "motor_status"
statusLevelStr = "level_status"
lcdRpmNum = "rpm_display"
lcdLevelNum = "level_display"
lcdParabolaNum = "parabola_display"
lcdAccelVectorNum = "gravity_display"
# ROS2 Publish Topic name
btnTopic = 'buttons_topic'
# ROS2 Publish mgs
msgBtnInit_init = "init"
msgBtnInit_start = "start_motors"
msgBtnInit_stop = "kill"
msgBtnProject_start = "start_proj"
msgBtnProject_stop = "stop_proj"
msgBtnPause_pause = "pause"
msgBtnPause_resume = "resume"

###### ******************* StyleSheets Variabels ************************* ######

styleSheet = """
QPushButton {
    padding:0.3em 1.2em;
    margin:0 0.1em 0.1em 0;
    border:0.16em solid rgba(255,255,255,0);
    border-radius: 20%;
    text-decoration:none;
    color:#FFFFFF;
    text-align:center;
}

QPushButton[text="Start Projection"], QPushButton[text="Options"],
QPushButton[text="Start Run"] {
    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                      stop: 0 #4e9af1, stop: 1 #0762d7);
}

QPushButton[text="Start Projection"]:pressed, QPushButton[text="Options"]:pressed,
QPushButton[text="Start Run"]:pressed {
    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                      stop: 0 #0762d7, stop: 1 #4e9af1);
}

QPushButton[text="Pause"], QPushButton[text="Stop Projection"],
QPushButton[text="Stop Run"] {
    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                       stop: 0 #ff4040, stop: 1 #992626);
}

QPushButton[text="Pause"]:pressed, QPushButton[text="Stop Projection"]:pressed,
QPushButton[text="Stop Run"]:pressed {
    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                       stop: 0 #992626, stop: 1 #ff4040);
}

QPushButton[text="Initialize Run"] {
    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                      stop: 0 #5bc11c, stop: 1 #2d600e);
}

QPushButton[text="Initialize Run"]:pressed {
    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                      stop: 0 #2d600e, stop: 1 #5bc11c);
}

QPushButton[enabled="false"] {
    background-color: gray;
}

QMessageBox {
    padding:0.3em 1.2em;
    margin:0 0.1em 0.1em 0;
    border:0.16em solid rgba(255,255,255,0);
    border-radius: 20%;
    text-decoration:none;
    color:#FFFFFF;
    text-align:center;
}

"""
msgStyleSheet = """

QLabel {
    font-size: 30px; 
    text-align:center;
}
QWidget icon{
    heigh: 60px;
}
QPushButton {
    padding:0.3em 1.2em;
    margin:0 0.1em 0.1em 0;
    border:0.16em solid rgba(255,255,255,0);
    border-radius: 20%;
    text-decoration:none;
    color:#FFFFFF;
    text-align:center;
    font-size: 40px;
}
QPushButton[text="OK"], QPushButton[text="Resume"] {
    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                      stop: 0 #4e9af1, stop: 1 #0762d7);
}
QPushButton[text="OK"]:pressed, QPushButton[text="Resume"]:pressed {
    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                      stop: 0 #0762d7, stop: 1 #4e9af1);
}
QPushButton[text="Cancel"] {
    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                      stop: 0 #b4bec3, stop: 1 #484c4e);
}
QPushButton[text="Cancel"]:pressed {
    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                      stop: 0 #484c4e, stop: 1 #b4bec3);
}
"""


class WorkerSignals(QObject):
    lcdRpm = pyqtSignal(int)
    lcdLevel = pyqtSignal(int)
    lcdParabola = pyqtSignal(int)
    lcdAccelVector = pyqtSignal(float)


class Worker(QRunnable):

    def __init__(self, fn):
        super(Worker, self).__init__()

        self.fn = fn
        self.signals = WorkerSignals()

    @pyqtSlot()
    def run(self):
        try:
            self.fn()
        except:
            traceback.print_exc()


class UI(QMainWindow):

    # This function sets initial gui state
    def __init__(self):
        super(UI, self).__init__()
        script_dir = os.path.dirname(__file__)
        # Load GUI design into python
        uic.loadUi(script_dir + "/spaceCalMW.ui", self)
        # apply CSS styleSheets to the GUI
        self.updateStyleSheet()

        # Use the following objectNames to acces GUI properties:
        """
        QPushButton:
            btnInit
            btnProject
            btnOptions
            btnPause

        QLabels:
            statusProjector
            statusMotor
            statusLevel
        
        QLCDNumber:
            lcdRpm
            lcdLevel
            lcdParabola
            lcdAccelVector
        """

        ##### Connect buttons to callback functions #####
        self.btnInit.clicked.connect(self.onClick_btnInit)
        self.btnProject.clicked.connect(self.onClick_btnProject)
        self.btnOptions.clicked.connect(self.onClick_btnOptions)
        self.btnPause.clicked.connect(self.onClick_btnPause)

        ##### Create confirmation windows #####
        # Question message
        self.msgConfirm = QMessageBox()
        self.msgConfirm.setIcon(QMessageBox.Question)
        self.msgConfirm.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        self.msgConfirm.setDefaultButton(QMessageBox.Ok)
        self.msgConfirm.buttons()[0].setText('OK')
        self.msgConfirm.buttons()[1].setText('Cancel')
        self.msgConfirm.setStyleSheet(msgStyleSheet)
        print(self.msgConfirm.buttons()[0].text())
        print(self.msgConfirm.buttons()[1].text())

        # Information message
        self.msgInfo = QMessageBox()
        self.msgInfo.setIcon(QMessageBox.Information)
        self.msgInfo.setStandardButtons(QMessageBox.Ok)
        self.msgInfo.setDefaultButton(QMessageBox.Ok)
        self.msgInfo.buttons()[0].setText('OK')
        self.msgInfo.setStyleSheet(msgStyleSheet)
        print(self.msgInfo.buttons()[0].text())
        # self.show()

        ###### ROS2 init #####
        # Initialize rospy
        rclpy.init(args=None)
        self.node = Node(pubNodeStr)
        self.pub = self.node.create_publisher(String, btnTopic, 10)

        # creating a multithread pool
        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" %
              self.threadpool.maxThreadCount())

        # create new thread for subcriber node
        worker = Worker(self.exec_subNode)
        # Execute
        self.threadpool.start(worker)

# ******************************* Buttons CallBack Function Definitions ******************************* #

    # This function defines the logic for the btnInit button.
    def onClick_btnInit(self):
        if self.btnInit.text() == runBtnInit:
            self.execBtnInit_init()
        elif self.btnInit.text() == runBtnStart:
            self.execBtnInit_start()
        elif self.btnInit.text() == runBtnStop:
            self.execBtnInit_stop()
        self.updateStyleSheet()

    # This function defines the logic for the btnProject button.
    def onClick_btnProject(self):
        if self.btnProject.text() == projectBtnStart:
            self.execBtnProject_start()
        elif self.btnProject.text() == projectBtnStop:
            self.execBtnProject_stop(True)
        self.updateStyleSheet()

    # This function defines the logic for the btnOptions button.
    def onClick_btnOptions(self):
        pass

    # This function defines the logic for the btnPause button.
    def onClick_btnPause(self):
        self.execBtnPause()
        pass


# ******************************** Button Functionality Functions **************************************** #

# The following function define the logic for all button states in the gui

    def execBtnInit_init(self):
        # Set the message for the information text
        self.displayInfoMsg(initRunMsg)
        # Pubish to ros2 topic (Initialize Printing Process)
        retPub = self.publishBtnInit(runBtnInit)
        # verify that motors were set to home
        if retPub == True:
            print("Motors Set to Home Successfully!!")
            # set btnInit new text
            self.btnInit.setText(runBtnStart)

    def execBtnInit_start(self):
        # Set the message for the confirmation text
        retMsg = self.displayConfirmatonMsg(startRunMsg)
        if retMsg == QMessageBox.Ok:
            # Publish to ros2 topic
            retPub = self.publishBtnInit(runBtnStart)
            # Confirm message was publish succesfully
            if retPub == True:
                print("Motors started succesfuly!!")
                # Set btnInit new text
                self.btnInit.setText(runBtnStop)
                self.btnProject.setEnabled(True)
                self.btnPause.setEnabled(True)

    def execBtnInit_stop(self):
        # Set the message for the confirmation text
        retMsg = self.displayConfirmatonMsg(stopRunMsg)
        if retMsg == QMessageBox.Ok:
            # Confirm selection
            retMsg = self.displayConfirmatonMsg(confStopRunMsg)
            if retMsg == QMessageBox.Ok:
                # Publish to ros2 topic
                retPub = self.publishBtnInit(runBtnStop)
                # Confirm message was publish succesfully
                if retPub == True:
                    print("Motors stoped succesfuly!!")
                    self.btnInit.setText(runBtnStart)
                    if self.btnProject.text() == projectBtnStop:
                        self.execBtnProject_stop(False)
                    self.btnPause.setEnabled(False)

    def execBtnProject_start(self):
        # Set the message for the confirmation text
        retMsg = self.displayConfirmatonMsg(startProjectionMsg)
        if retMsg == QMessageBox.Ok:
            # Publish to ros2 topic
            retPub = self.publishBtnProject(projectBtnStart)
            # Confirm message was publish succesfully
            if retPub == True:
                print("Projector Started Succesfuly!!")
                self.btnProject.setText(projectBtnStop)
                self.btnPause.setEnabled(True)

    def execBtnProject_stop(self, displayMsg):
        msg = True
        if displayMsg:
            # Set the message for the confirmation text
            retMsg = self.displayConfirmatonMsg(stopProjectionMsg)
            if retMsg != QMessageBox.Ok:
                msg = False
        if msg:
            # Publish to ros2 topic
            retPub = self.publishBtnProject(projectBtnStop)
            # Confirm message was publish succesfully
            if retPub == True:
                print("Projector Stoped Succesfuly!!")
                self.btnProject.setText(projectBtnStart)
        if self.btnInit.text() == runBtnStart:
            self.btnPause.setEnabled(False)

    def execBtnPause(self):
        retMsg = self.displayConfirmatonMsg(pauseAllMsg)
        if retMsg == QMessageBox.Ok:
            # Publish to ros2 topic
            retPub = self.publishBtnPause(pauseBtnPause)
            # Confirm message was publish succesfully
            if retPub == True:
                print("Projector and Motors Paused Succesfuly!!")
                retMsg = self.displayInfoMsg(pauseInfoMsg)
                if retMsg == QMessageBox.Ok:
                    retPub = self.publishBtnPause(pauseBtnResume)

    def execBtnOptions(self):
        pass

# ********************************* Labels and LCDNumber Setter Functions ******************************** #
    """
    QLabels:
        statusProjector
        statusMotor
        statusLevel

    QLCDNumber:
        lcdRpm
        lcdLevel
        lcdParabola
        lcdAccelVector
    """

    def setStatusProjectorDisplay(self, str):
        print(str)
        self.statusProjector.setText(str.data)

    def setStatusMotorDisplay(self, str):
        self.statusMotor.setText(str.data)

    def setStatusLevelDisplay(self, str):
        self.statusLevel.setText(str.data)

    def setLcdRpmDisplay(self, num):
        print(num)
        self.lcdRpm.display(num.data)

    def setLcdLevelDisplay(self, num):
        self.lcdLevel.display(num.data)

    def setLcdParabolaDisplay(self, num):
        self.lcdParabola.display(num.data)

    def setLcdAccelVectorDisplay(self, num):
        self.lcdAccelVector.display(num.data)


# *************************************** Define Publisher Functions ************************************** #
    # this funtion publishes messages from the btninit button.


    def publishBtnInit(self, str):
        msg = String()
        if str == runBtnInit:
            msg.data = msgBtnInit_init
        elif str == runBtnStart:
            msg.data = msgBtnInit_start
        elif str == runBtnStop:
            msg.data = msgBtnInit_stop
        self.pub.publish(msg)
        return True

    def publishBtnProject(self, str):
        msg = String()
        if str == projectBtnStart:
            msg.data = msgBtnProject_start
        elif str == projectBtnStop:
            msg.data = msgBtnProject_stop
        self.pub.publish(msg)
        return True

    def publishBtnPause(self, str):
        msg = String()
        if str == pauseBtnPause:
            msg.data = msgBtnPause_pause
        elif str == pauseBtnResume:
            msg.data = msgBtnPause_resume
        self.pub.publish(msg)
        return True


# *************************************** Define Subscriber Functions ************************************* #
    """
    QPushButton:
        btnInit
        btnProject
        btnOptions
        btnPause

    QLabels:
        statusProjector
        statusMotor
        statusLevel
    
    QLCDNumber:
        lcdRpm
        lcdLevel
        lcdParabola
        lcdAccelVector
    """

    def exec_subNode(self):
        node = Node(subNodeStr)
        subProjStatus = node.create_subscription(
            String, statusProjectorStr, self.setStatusProjectorDisplay, 10)
        subMotorStatus = node.create_subscription(
            String, statusMotorStr, self.setStatusMotorDisplay, 10)
        subLelelStatus = node.create_subscription(
            String, statusLevelStr, self.setStatusLevelDisplay, 10)
        subRpm = node.create_subscription(
            Int32, lcdRpmNum, self.setLcdRpmDisplay, 10)
        subLevel = node.create_subscription(
            Int32, lcdLevelNum, self.setLcdLevelDisplay, 10)
        subParabola = node.create_subscription(
            Int32, lcdParabolaNum, self.setLcdParabolaDisplay, 10)
        subAccelVec = node.create_subscription(
            Int32, lcdAccelVectorNum, self.setLcdAccelVectorDisplay, 10)
        rclpy.spin(node)
        node.destroy_node()

# ******************************************* Helper Functions ******************************************** #

    def updateStyleSheet(self):
        self.setStyleSheet(styleSheet)

    def displayInfoMsg(self, msg):
        self.msgInfo.setText(msg)
        return self.msgInfo.exec()

    def displayConfirmatonMsg(self, msg):
        self.msgConfirm.setText(msg)
        return self.msgConfirm.exec()


def main():
    # Set the proper OS variable to display on
    os.environ['DISPLAY'] = ":0"
    app = QApplication(sys.argv)
    window = UI()
    window.show()
    app.exec()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
