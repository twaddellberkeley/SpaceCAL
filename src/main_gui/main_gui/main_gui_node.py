
import threading
import os
import rclpy
from rclpy.node import Node
import time
from threading import Thread
from functools import partial
from queue import Queue

import sys

from PyQt5.QtWidgets import (QApplication)

from .submodules.mainwindow import MainWindow
from interfaces.srv import GuiDisplay, GuiInput, Projector, Video, MotorSrv

gui_inpu_srv = 'gui_input_srv'
gui_display_srv = 'gui_display_srv'


class GuiNode(Node):

    def __init__(self, window):
        super().__init__('dummy')
        # 'main_gui_server_node'
        self.gui = window
        self.gui.guiLogic.logger = self.get_logger()
        # self.gui.guiLogic.cli_req = self.client_req
        # self.cli = self.create_client(GuiInput, gui_inpu_srv)
        serverThread = threading.Thread(target=self.exec_gui_node)
        serverThread.start()

    def exec_gui_node(self):
        subNode = Node('main_gui_server_display_node')
        #***** Servers ******#
        server = subNode.create_service(
            GuiDisplay, gui_display_srv, self.gui_display_callback)
        rclpy.spin(subNode)
        subNode.destroy_node()

    def gui_display_callback(self, req, res):

        # Metadata meta
        # int32 printer_id
        # string cmd
        # int32 state
        # string display_name
        # string display_msg
        # string[] pi_queue
        # string[] pi_videos
        self.gui.guiLogic.serverData(req)

        # if "display" in req.cmd:
        #     p = str(req.printer_id)
        #     if req.printer_id < 0:
        #         p = "all"
        #     self.gui.guiLogic.updateGuiDisplay(
        #         req.display_name + "-" + req.display_msg + "-" + p)
        # if "state" in req.cmd:
        #     self.gui.guiLogic.updateGuiState(req.state)
        # if "videos" in req.cmd:
        #     self.gui.guiLogic.setVideos(req.printer_id, req.pi_videos)
        # if "queue" in req.cmd:
        #     self.gui.guiLogic.setQueue(req.printer_id, req.pi_queue)

        print(req.cmd)
        res.err = 0
        res.cmd = req.cmd
        return res


def main(args=None):
    os.environ['DISPLAY'] = ":0"
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    window = MainWindow()
    gui = GuiNode(window)
    window.show()

    # window.pubNode.destroy_node()

    # rclpy.shutdown()
    app.exec()
    gui.destroy_node()
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
