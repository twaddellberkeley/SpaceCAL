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
from interfaces.msg import DisplayData

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
# btn 1
runBtnInit = "Initialize Run"
runBtnStart = "Start Run"
runBtnStop = "Stop Run"
# btn 2
projectBtnStart = "Start Projection"
projectBtnStop = "Stop Projection"
# btn 3
optionBtn = "Options"
# btn 4
pauseBtnPause = "Pause"
pauseBtnResume = "Resume"

# Display labels
projStatusArr = ['Off', 'On']
motorStatusArr = ['Idle', 'Rotating', 'Stopped', 'Pause']
levelStatusArr = ['Idle', 'Homing', 'Homed', 'Moving' 'Set']

###### *********************** ROS2 Variables **************************** ######
# ROS2 Nodes names
pubNodeStr = "buttons_node"
subNodeStr = "display_node"
# ROS2 Subscriber Topic name
displayTopic = "display_topic"
# Display label names:
statusProjectorStr = "projector_status"
statusMotorStr = "motor_status"
statusLevelStr = "level_status"
lcdRpmNum = "rpm_display"
lcdLevelNum = "level_display"
lcdParabolaNum = "parabola_display"
lcdAccelVectorNum = "gravity_display"
# Reset run
resetRun = "reset_run"
resetProjection = "reset_projection"
# ROS2 Publisher Topic name
btnTopic = 'buttons_topic'
# ROS2 Publishing mgs for "buttons_topic"
msgBtnInit_init = "motor_ok"
msgBtnInit_start = "motor_ok"
msgBtnInit_stop = "kill"
msgBtnProject_start = "start_proj"
msgBtnProject_stop = "stop_proj"
msgBtnPause_pause = "pause"
msgBtnPause_resume = "resume"

###### ******************* StyleSheets Variabels ************************* ######
# global styleSheet
# styleSheet = """
# QPushButton {
#     padding:0.3em 1.2em;
#     margin:0 0.1em 0.1em 0;
#     border:0.16em solid rgba(255,255,255,0);
#     border-radius: 20%;
#     text-decoration:none;
#     color:#FFFFFF;
#     text-align:center;
# }

# QPushButton[text="Start Projection"], QPushButton[text="Options"],
# QPushButton[text="Start Run"] {
#     background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                       stop: 0 #4e9af1, stop: 1 #0762d7);
# }

# QPushButton[text="Start Projection"]:pressed, QPushButton[text="Options"]:pressed,
# QPushButton[text="Start Run"]:pressed {
#     background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                       stop: 0 #0762d7, stop: 1 #4e9af1);
# }

# QPushButton[text="Pause"], QPushButton[text="Stop Projection"],
# QPushButton[text="Stop Run"] {
#     background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                        stop: 0 #ff4040, stop: 1 #992626);
# }

# QPushButton[text="Pause"]:pressed, QPushButton[text="Stop Projection"]:pressed,
# QPushButton[text="Stop Run"]:pressed {
#     background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                        stop: 0 #992626, stop: 1 #ff4040);
# }

# QPushButton[text="Initialize Run"] {
#     background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                       stop: 0 #5bc11c, stop: 1 #2d600e);
# }

# QPushButton[text="Initialize Run"]:pressed {
#     background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                       stop: 0 #2d600e, stop: 1 #5bc11c);
# }

# QPushButton[enabled="false"] {
#     background-color: gray;
# }

# QMessageBox {
#     padding:0.3em 1.2em;
#     margin:0 0.1em 0.1em 0;
#     border:0.16em solid rgba(255,255,255,0);
#     border-radius: 20%;
#     text-decoration:none;
#     color:#FFFFFF;
#     text-align:center;
# }

# """
# msgStyleSheet = """

# QLabel {
#     font-size: 30px;
#     text-align:center;
# }
# QWidget icon{
#     heigh: 60px;
# }
# QPushButton {
#     padding:0.3em 1.2em;
#     margin:0 0.1em 0.1em 0;
#     border:0.16em solid rgba(255,255,255,0);
#     border-radius: 20%;
#     text-decoration:none;
#     color:#FFFFFF;
#     text-align:center;
#     font-size: 40px;
# }
# QPushButton[text="OK"], QPushButton[text="Resume"] {
#     background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                       stop: 0 #4e9af1, stop: 1 #0762d7);
# }
# QPushButton[text="OK"]:pressed, QPushButton[text="Resume"]:pressed {
#     background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                       stop: 0 #0762d7, stop: 1 #4e9af1);
# }
# QPushButton[text="Cancel"] {
#     background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                       stop: 0 #b4bec3, stop: 1 #484c4e);
# }
# QPushButton[text="Cancel"]:pressed {
#     background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
#                                       stop: 0 #484c4e, stop: 1 #b4bec3);
# }
# """


class WorkerSignals(QObject):
    btnChange = pyqtSignal()
    processRecieveMsg = pyqtSignal(DisplayData)
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

        ####### Create state signals ########
        # self.lcdRpm.changeEvent(self.updateBtnState)

        ##### Create confirmation windows #####
        # Question message
        self.msgConfirm = QMessageBox()
        self.msgConfirm.setIcon(QMessageBox.Question)
        self.msgConfirm.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        self.msgConfirm.setDefaultButton(QMessageBox.Ok)
        self.msgConfirm.buttons()[0].setText('OK')
        self.msgConfirm.buttons()[1].setText('Cancel')
        self.msgConfirm.setStyleSheet(self.msgStyleSheet)
        print(self.msgConfirm.buttons()[0].text())
        print(self.msgConfirm.buttons()[1].text())

        # Information message
        self.msgInfo = QMessageBox()
        self.msgInfo.setIcon(QMessageBox.Information)
        self.msgInfo.setStandardButtons(QMessageBox.Ok)
        self.msgInfo.setDefaultButton(QMessageBox.Ok)
        self.msgInfo.buttons()[0].setText('OK')
        self.msgInfo.setStyleSheet(self.msgStyleSheet)
        print(self.msgInfo.buttons()[0].text())
        # self.show()

        ###### ************************************* ROS2 init ************************************* #####
        # Initialize rospy
        rclpy.init(args=None)
        self.pubNode = Node(pubNodeStr)
        self.pub = self.pubNode.create_publisher(String, btnTopic, 10)
        self.pubDisplay = self.pubNode.create_publisher(
            DisplayData, displayTopic, 10)

        # creating a multithread pool
        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" %
              self.threadpool.maxThreadCount())

        # create new thread for subcriber node
        self.worker = Worker(self.exec_subNode)
        self.worker.signals.btnChange.connect(self.updateStyleSheet)
        self.worker.signals.processRecieveMsg.connect(self.subcriberHelper)
        # Execute
        self.threadpool.start(self.worker)

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
        ####### Pubish to ros2 topic (Initialize Printing Process) #######
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
            ###### Publish to ros2 topic #######
            retPub = self.publishBtnInit(runBtnStart)
            # Confirm message was publish succesfully
            if retPub == True:
                print("Motors started succesfuly!!")
                # Set btnInit new text
                self.btnInit.setText(runBtnStop)
                self.btnProject.setEnabled(True)
                # self.btnPause.setEnabled(True)

    def execBtnInit_stop(self):
        # Set the message for the confirmation text
        retMsg = self.displayConfirmatonMsg(stopRunMsg)
        if retMsg == QMessageBox.Ok:
            # Confirm selection
            retMsg = self.displayConfirmatonMsg(confStopRunMsg)
            if retMsg == QMessageBox.Ok:
                ####### Publish to ros2 topic #######
                retPub = self.publishBtnInit(runBtnStop)
                # Confirm message was publish succesfully
                if retPub == True:
                    print("Motors stoped succesfuly!!")
                    self.btnInit.setText(runBtnStart)
                    if self.btnProject.text() == projectBtnStop:
                        self.execBtnProject_stop(False)
                    # self.btnPause.setEnabled(False)

    def execBtnProject_start(self):
        # Set the message for the confirmation text
        retMsg = self.displayConfirmatonMsg(startProjectionMsg)
        if retMsg == QMessageBox.Ok:
            ###### Publish to ros2 topic #######
            retPub = self.publishBtnProject(projectBtnStart)
            # Confirm message was publish succesfully
            if retPub == True:
                print("Projector Started Succesfuly!!")
                self.btnProject.setText(projectBtnStop)
                # self.btnPause.setEnabled(True)

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
        # if self.btnInit.text() == runBtnStart:
        #     self.btnPause.setEnabled(False)

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
        if str == projStatusArr[0]:  # on
            if self.btnProject.Text() != projectBtnStop:
                self.btnProject.setText(projectBtnStop)
                # self.btnPause.setEnabled(True)
        elif str == projStatusArr[1]:  # off
            if self.btnProject.Text() != projectBtnStart:
                self.btnProject.setText(projectBtnStop)
                # self.btnPause.setEnabled(True)
        self.statusProjector.setText(str)
        self.updateStyleSheet()

    # motorStatusArr = ['Idle', 'Rotating', 'Stopped']
    def setStatusMotorDisplay(self, str):
        if str == motorStatusArr[0]:  # Idle
            pass
        elif str == motorStatusArr[1]:  # Rotating
            pass
        elif str == motorStatusArr[2]:  # Stopped
            self.btnProject.setText(projectBtnStart)
            self.btnInit.setText(runBtnStart)
            self.btnProject.setEnable(False)
            pass
        self.statusMotor.setText(str)
        self.updateStyleSheet()

    # levelStatusArr = ['Idle', 'Homing', 'Homed', 'Moving' 'Set']
    def setStatusLevelDisplay(self, str):
        if str == levelStatusArr[0]:    # Idle
            pass
        elif str == levelStatusArr[1]:  # Homing
            self.btnProject.setEnable(False)
            self.btnInit.setEnable(False)
            pass
        elif str == levelStatusArr[2]:  # Homed
            self.btnProject.setEnable(True)
            self.btnInit.setEnable(True)
            pass
        elif str == levelStatusArr[3]:  # Moving
            self.btnProject.setEnable(False)
            pass
        elif str == levelStatusArr[4]:  # Set
            self.btnProject.setEnable(True)
            pass
        self.statusLevel.setText(str)
        self.updateStyleSheet()

    def setLcdRpmDisplay(self, num):
        print(num)
        self.lcdRpm.display(num)

    def setLcdLevelDisplay(self, num):
        self.lcdLevel.display(num)

    def setLcdParabolaDisplay(self, num):

        self.lcdParabola.display(num)

    def setLcdAccelVectorDisplay(self, num):
        self.lcdAccelVector.display(num)


# *************************************** Define Publisher Functions ************************************** #
    # this funtion publishes messages from the btninit button.


    def publishBtnInit(self, str):
        msg = String()
        dis = DisplayData()
        if str == runBtnInit:
            msg.data = msgBtnInit_init
            ####### publish motors "On" #######
            dis.name = statusMotorStr
            dis.str_value = motorStatusArr[0]
            self.pubDisplay.publish(dis)
        elif str == runBtnStart:
            msg.data = msgBtnInit_start
        elif str == runBtnStop:
            msg.data = msgBtnInit_stop
        self.pub.publish(msg)
        return True

    def publishBtnProject(self, str):
        msg = String()
        dis = DisplayData()
        if str == projectBtnStart:
            msg.data = msgBtnProject_start
            ####### publish projection "On" ########
            dis.name = statusProjectorStr
            dis.str_value = projStatusArr[0]
            self.pubDisplay.publish(dis)
        elif str == projectBtnStop:
            msg.data = msgBtnProject_stop
            ####### publish projection "Off" ########
            dis.name = statusProjectorStr
            dis.str_value = projStatusArr[1]
            self.pubDisplay.publish(dis)
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

# *************************************** Define Subscriber Functions ************************************** #

    def subcriberNodeHandler(self, data):
        self.worker.signals.processRecieveMsg(data)

    def subcriberHelper(self, data):
        print("got")
        if data.name == statusProjectorStr:
            self.setStatusProjectorDisplay(data.str_value)
        elif data.name == statusMotorStr:
            self.setStatusMotorDisplay(data.str_value)
        elif data.name == statusLevelStr:
            self.setStatusLevelDisplay(data.str_value)
        elif data.name == lcdRpmNum:
            self.setLcdRpmDisplay(data.num_value)
        elif data.name == lcdLevelNum:
            self.setLcdLevelDisplay(data.num_value)
        elif data.name == lcdParabolaNum:
            self.setLcdParabolaDisplay(data.num_value)
        # NEW TODO: delete the bottom 2 if stattements
        elif data.name == resetRun:
            self.resetGuiRun()
        elif data.name == resetProjection:
            self.resetProjection()
        else:
            print("No label with name: " + data.name)


# *************************************** Define Subscriber Node function ************************************* #
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
        subNode = Node(subNodeStr)
        sub = subNode.create_subscription(
            DisplayData,
            displayTopic,
            self.subcriberNodeHandler, 10)

        # subProjStatus = subNode.create_subscription(
        #     String, statusProjectorStr, self.setStatusProjectorDisplay, 10)
        # subMotorStatus = subNode.create_subscription(
        #     String, statusMotorStr, self.setStatusMotorDisplay, 10)
        # subLelelStatus = subNode.create_subscription(
        #     String, statusLevelStr, self.setStatusLevelDisplay, 10)
        # subRpm = subNode.create_subscription(
        #     Int32, lcdRpmNum, self.setLcdRpmDisplay, 10)
        # subLevel = subNode.create_subscription(
        #     Int32, lcdLevelNum, self.setLcdLevelDisplay, 10)
        # subParabola = subNode.create_subscription(
        #     Int32, lcdParabolaNum, self.setLcdParabolaDisplay, 10)
        # subAccelVec = subNode.create_subscription(
        #     Int32, lcdAccelVectorNum, self.setLcdAccelVectorDisplay, 10)
        rclpy.spin(subNode)
        subNode.destroy_node()

# ******************************************* Helper Functions ******************************************** #

    def updateStyleSheet(self):
        self.setStyleSheet(self.styleSheet)

    def displayInfoMsg(self, msg):
        self.msgInfo.setText(msg)
        return self.msgInfo.exec()

    def displayConfirmatonMsg(self, msg):
        self.msgConfirm.setText(msg)
        return self.msgConfirm.exec()

    # NEW

    def updateBtnState(slef):
        pass

    def resetGuiRun(self):
        self.btnInit.setText(runBtnStart)
        self.worker.signals.btnChange.emit()
        # self.updateStyleSheet()

    def resetProjection(self):
        self.btnProject.setText(projectBtnStart)
        self.worker.signals.btnChange.emit()
        # self.updateStyleSheet()


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
