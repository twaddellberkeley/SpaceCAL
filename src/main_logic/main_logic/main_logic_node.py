
import time
from threading import Thread
import threading
from functools import partial
from queue import Queue

from interfaces.srv import GuiDisplay, GuiInput, Projector, Video, MotorSrv
from .submodules.level_controller import LevelController
from .submodules.printer_controller import PrinterController


import rclpy
from rclpy.node import Node

PRINT_TIME = 20


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

        # create an instance of an event
        # Ok to run is for waitng for the level platform to stop moving
        self.okToRunEvent = threading.Event()
        self.stopAllEvent = threading.Event()

        #***** Initialize Level Controller ******#
        self.levelController = LevelController(self.okToRunEvent)

        self.ledOffThread = None

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
                 "proj_cmds": [], "pi_cmds": [], "custom_cmd": []}
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
            else:
                # This are custom commands
                if cmd in ["start-run", "stop-run"]:
                    ctrls["custom_cmd"].append(cmd)

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
                # # Wait for the platform to stop moving
                # while self.levelController._is_moving:
                #     time.sleep(.5)

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
                    self.printerController[printer].sen_proj_cmd(cmd)

        if len(ctrls["pi_cmds"]) != 0:
            # Send commands to pi controller logic
            for cmd in ctrls["pi_cmds"]:
                # Find out who is the message for
                dest = cmd.split("-")[-1]
                printers = self.get_printers_num(dest)
                for printer in printers:
                    self.printerController[printer].send_pi_cmd(cmd)

        if len(ctrls["custom_cmd"]) != 0:
            # Send custom command to all systems
            for cmd in ctrls["custom_cmd"]:
                if cmd == "stop-run":
                    self.send_stop_run_cmd()
                elif cmd == "start-run":
                    self.send_start_run_cmd()
                elif cmd == "continue-run":
                    self.send_continue_run_cmd()
                elif cmd == "start-print":
                    self.send_start_print_cmd()
                elif cmd == "stop-print":
                    self.send_stop_print_cmd()

        ###################################################################################
        self.get_logger().debug('Finished Dispatching Gui Commands...\n')    # DEBUG MESSGE
        ###################################################################################
        return 0

    def send_start_run_cmd(self):
        self.okToRunEvent.set()
        startRunThread = threading.Thread(target=self.start_run())

        # Allow the thread to run in the backgorund
        startRunThread.daemon = True
        startRunThread.start()

    def send_stop_run_cmd(self):
        self.okToRunEvent.clear()
        self.stopAllEvent.set()
        self.turn_off_all_systems()

    def send_continue_run_cmd(self):
        # Move leve to the next leve
        self.okToRunEvent.clear()
        printers = self.get_printers_num("all")
        if self.stopAllEvent.is_set():
            return 1
        next_level = (self.levelController._curr_level + 1) % 5
        self.levelController.send_level_cmd("level-motors-" + str(next_level))
        self.okToRunEvent.wait()
        if self.stopAllEvent.is_set():
            return 1
        for printer in printers:
            self.printerController[printer].send_pi_cmd("pi-play-queue-all")
            self.printerController[printer].send_motor_cmd("motor-on-9-all")
        if self.stopAllEvent.is_set():
            for printer in printers:
                self.printerController[printer].send_proj_cmd(
                    "proj-led-off-all")
                self.printerController[printer].send_pi_cmd(
                    "pi-stop-queue-all")
                self.printerController[printer].send_motor_cmd("motor-off-all")
            return 1

    def send_start_print_cmd(self):
        self.stopAllEvent.clear()
        printers = self.get_printers_num("all")
        for printer in printers:
            self.printerController[printer].send_proj_cmd("proj-led-on-all")
        self.ledOffThread = threading.Timer(
            PRINT_TIME, self.send_stop_print_cmd)
        self.ledOffThread.start()

    def send_stop_print_cmd(self):
        if self.ledOffThread != None:
            self.ledOffThread.cancel()
        printers = self.get_printers_num("all")
        for printer in printers:
            self.printerController[printer].send_proj_cmd("proj-led-off-all")
            self.printerController[printer].send_pi_cmd("pi-stop-video-all")

    def start_run(self):
        self.stopAllEvent.clear()
        printers = self.get_printers_num("all")
        for printer in printers:
            self.printerController[printer].send_proj_cmd("proj-led-off-all")
            self.printerController[printer].send_pi_cmd("pi-stop-video-all")
            self.printerController[printer].send_motor_cmd("motor-off-all")
            self.printerController[printer].send_pi_cmd("pi-reset-queue-all")

        # Move motors and wait for them to signal to continue
        self.okToRunEvent.clear()
        if self.stopAllEvent.is_set():
            return 1
        self.levelController.send_level_cmd("level-motors-home")
        self.okToRunEvent.wait()
        self.okToRunEvent.clear()
        if self.stopAllEvent.is_set():
            return 1
        self.levelController.send_level_cmd("level-motors-0")
        self.okToRunEvent.wait()
        if self.stopAllEvent.is_set():
            return 1
        for printer in printers:
            self.printerController[printer].send_pi_cmd("pi-play-queue-all")
            self.printerController[printer].send_motor_cmd("motor-on-9-all")
        if self.stopAllEvent.is_set():
            for printer in printers:
                self.printerController[printer].send_proj_cmd(
                    "proj-led-off-all")
                self.printerController[printer].send_pi_cmd(
                    "pi-stop-queue-all")
                self.printerController[printer].send_motor_cmd("motor-off-all")
            return 1

    def start_print(self):

        pass

    ################################################ Client Request ##############################################

    # TODO: make sure to remember to update the project and motor classes from the response data

    def get_printers_num(self, dest):
        if dest == "all":
            return [0, 1, 2, 3, 4]
        return [int(dest)]

    def turn_off_all_systems(self):
        if self.levelController._is_moving:
            self.levelController.goal_handle.cancel_goal_async()
        for printer in range(5):
            self.printerController[printer].send_proj_cmd("proj-led-off-all")
            self.printerController[printer].send_pi_cmd("pi-stop-video-all")
            self.printerController[printer].send_motor_cmd("motor-off-all")


def main(args=None):
    rclpy.init(args=args)
    #print('In Main thread with thread_id: %d\n' % threading.get_native_id())
    service = MainLogicNode()
    # service.send_height_goal(10)

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
