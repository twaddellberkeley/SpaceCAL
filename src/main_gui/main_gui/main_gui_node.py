
import threading
import os
import rclpy
from rclpy.node import Node

import sys

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
                            QMetaObject, QObject, QPoint, QRect,
                            QSize, QTime, QUrl, Qt, Slot, Signal)

from PySide6.QtWidgets import (QApplication, QFrame, QHBoxLayout, QLabel,
                               QLayout, QMainWindow, QPushButton, QSizePolicy,
                               QStatusBar, QTabWidget, QVBoxLayout, QWidget)

from .submodules.mainwindow import MainWindow
from Interfaces.srv import GuiDisplay


class GuiNode():

    def __init__(self, window):
        super().__init__()
        rclpy.init(args=None)
        self.gui = window

    def exec_gui_node(self):
        subNode = Node('main_gui_server_node')
        #***** Servers ******#
        self.gui_srv = self.create_service(
            GuiDisplay, 'gui_input_srv', self.gui_input_callback)

        rclpy.spin(subNode)
        subNode.destroy_node()

    def gui_display_callback(slef):
        pass


def main(args=None):
    os.environ['DISPLAY'] = ":0"
    app = QApplication(sys.argv)
    window = MainWindow()
    gui = GuiNode(window)
    gui.gui.show()
    e = app.exec()
    window.pubNode.destroy_node()
    rclpy.shutdown()
    # rclpy.shutdown()
    sys.exit(e)

    # rclpy.init(args=args)
    # gui = GuiNode()
    # print('Hi from GUI %d.', gui.motor_num)
    # try:
    #     rclpy.spin(gui)
    # finally:
    #     gui.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
