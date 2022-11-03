
import threading
import os
import rclpy
from rclpy.node import Node

import sys

from PyQt5.QtWidgets import (QApplication)

from .submodules.mainwindow import MainWindow
from interfaces.srv import GuiDisplay, GuiInput, Projector, Video, MotorSrv


class GuiNode(Node):

    def __init__(self, window):
        super().__init__('main_gui_server_display_node')
        # 'main_gui_server_node'
        
        self.gui = window
        serverThread = threading.Thread(target=self.exec_gui_node)
        serverThread.start()
        

    def exec_gui_node(self):
        subNode = Node('main_gui_server_display_node')
        #***** Servers ******#
        server = subNode.create_service(
            GuiDisplay, 'gui_display_srv', self.gui_display_callback)
        rclpy.spin(subNode)
        subNode.destroy_node()
        

    def gui_display_callback(self, req, res):

        self.gui.sendCmd("signal_display_input")
        res.cmd = "Hello"
        return res
        


def main(args=None):
    os.environ['DISPLAY'] = ":0"
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    window = MainWindow()
    gui = GuiNode(window)
    gui.gui.show()
    e = app.exec()
    # window.pubNode.destroy_node()
   
    # rclpy.shutdown()
    sys.exit(e)
    rclpy.shutdown()
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
