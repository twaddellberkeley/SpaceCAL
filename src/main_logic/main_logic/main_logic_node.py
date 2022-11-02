
import time
from threading import Thread
import threading
from functools import partial
from queue import Queue

from interfaces.srv import GuiDisplay, GuiInput, Projector, Video, MotorSrv
from .submodule.main_logic import LevelController
from .submodule.main_logic import PrinterController


import rclpy
from rclpy.node import Node


class MainLogicNode(Node):

    def __init__(self):
        super().__init__('main_logic_node')

        ################## Services ####################
        #***** Servers ******#
        self.gui_srv = self.create_service(
            GuiInput, 'gui_input_srv', self.gui_input_callback)

        #***** Initialize printer controller objects ******#
        self.printerController = [PrinterController(0), PrinterController(
            1), PrinterController(2), PrinterController(3), PrinterController(4)]

        #***** Initialize Level Controller ******#
        self.levelController = LevelController()

    def gui_input_callback(self, request, response):
        self.get_logger().info('Incoming Gui Request\ncmd: %s ' % (request.cmd))
        # Decode gui input command
        cmd_list = self.decode_gui_cmd(request.cmd)
        # dispach the commands
        res = self.dispatch_gui_cmd(cmd_list)

        response.err = res
        response.msg = "request is is being proccess"
        return response

    def decode_gui_cmd(self, raw_cmd):
        self.get_logger().info('Decoding Gui Command:  %s\n' % raw_cmd)
        # Projector and Level Commands
        ctrls = {"level_cmds": [], "motor_cmds": [],
                 "proj_cmds": [], "pi_cmds": []}
        # Split comnands
        cmd_list = raw_cmd.split("_")
        for cmd in cmd_list:
            split_cmd = cmd.split("-")
            if "proj" == split_cmd[0]:
                # extract project command
                # ctrls["proj_cmds"].append(cmd[len("proj-"):len(cmd)])
                ctrls["proj_cmds"].append(cmd)
                ##################################################################################
                self.get_logger().debug('******** first cmd in proj_cmds: %s\n' %
                                        ctrls["proj_cmds"][0])  # DEBUG MESSAGE
                ##################################################################################
            elif "pi" == split_cmd[0]:
                # extract pi command
                ctrls["pi_cmds"].append(cmd)
                ##################################################################################
                self.get_logger().debug('******** first cmd in pi_cmds: %s\n' %
                                        ctrls["pi_cmds"][0])  # DEBUG MESSAGE
                ##################################################################################
            elif "motor" == split_cmd[0]:
                # extract motor command
                ctrls["motor_cmds"].append(cmd)
                ##################################################################################
                self.get_logger().debug("******** first cmd in motor_cmds: %s\n" %
                                        ctrls["motor_cmds"][0])  # DEBUG MESSAGE
                ##################################################################################
            elif "level" == split_cmd[0]:
                # extract level commands
                ctrls["level_cmds"].append(cmd)
                ##################################################################################
                self.get_logger().debug('******** first cmd in level_cmds: %s' %
                                        ctrls["level_cmds"][0])  # DEBUG MESSAGE
                ##################################################################################
        return ctrls

    def dispatch_gui_cmd(self, ctrls):
        """ TODO: Add fucntions description """
        # It reads from a list of commands and dispatches them in their own thread
        ########################################################################
        self.get_logger().debug('Dispatching Gui Commands...\n')  # DEBUG MESSGE
        ########################################################################

        if len(ctrls["level_cmds"]) != 0:
            # Send command to level controller logic
            for cmd in ctrls["level_cmds"]:
                self.levelController.send_level_cmd(cmd)
                # Wait for the platform to stop moving
                while self.levelController._is_moving:
                    time.sleep(.5)

        if len(ctrls["motor_cmds"]) != 0:
            # Send commadns to motor controller logic
            for cmd in ctrls["motor_cmds"]:
                # Find out who is the message for
                dest = cmd.split("-")[-1]
                printers = self.get_printers_num(dest)
                for printer in printers:
                    self.printerController[printer].send_motor_cmd(cmd)

        if len(ctrls["proj_cmds"]) != 0:
            # Send commands to projector controller logic
            for cmd in ctrls["proj_cmds"]:
                # Find out who is the message for
                dest = cmd.split("-")[-1]
                printers = self.get_printers_num(dest)
                for printer in printers:
                    self.printerController[printer].proj_controller_logic(cmd)

        if len(ctrls["pi_cmds"]) != 0:
            # Send commands to pi controller logic
            for cmd in ctrls["pi_cmds"]:
                # Find out who is the message for
                dest = cmd.split("-")[-1]
                printers = self.get_printers_num(dest)
                for printer in printers:
                    self.printerController[printer].pi_controller_logic(cmd)

        ###################################################################################
        self.get_logger().debug('Finished Dispatching Gui Commands...\n')    # DEBUG MESSGE
        ###################################################################################
        return 0

    ################################################ Client Request ##############################################

    # TODO: make sure to remember to update the project and motor classes from the response data
    def get_printers_num(self, dest):
        if dest == "all":
            return [0, 1, 2, 3, 4]
        return [int(dest)]


def main(args=None):
    rclpy.init(args=args)
    #print('In Main thread with thread_id: %d\n' % threading.get_native_id())
    service = MainLogicNode()
    # service.send_height_goal(10)

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
