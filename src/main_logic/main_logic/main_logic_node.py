
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

PRINT_TIME = 9
testVar = "0"

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
        # self.printerController = [PrinterController(0)]

        # create an instance of an event
        # Ok to run is for waitng for the level platform to stop moving
        # self.okToRunEvent = threading.Event()
        # self.stopAllEvent = threading.Event()

        #***** Initialize Level Controller ******#
        self.levelController = LevelController(self)



    def gui_input_callback(self, request, response):
        self.get_logger().info('Incoming Gui Request\ncmd: %s ' % (request.cmd))
        # Decode gui input command
        cmd_list = self.decode_gui_cmd(request.cmd)
        res = -1
        # dispach the commands
        if cmd_list != None:
            res = self.dispatch_gui_cmd(cmd_list)


        response.err = res
        response.msg = "request is is being proccess"
        return response

    def decode_gui_cmd(self, raw_cmd):
        if raw_cmd == None:
            return None
        self.get_logger().info('Decoding Gui Command:  %s\n' % raw_cmd)
        # Projector and Level Commands
        ctrls = {"level_cmds": [], "motor_cmds": [],
                 "proj_cmds": [], "pi_cmds": [], "custom_cmd": []}
        # Split comnands
        
        cmd_list = raw_cmd.split("+")
        
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
                self.get_logger().error('Command in:  %s\n' % cmd)
                if cmd in ["start-run", "stop-run", "continue-run", "start-print", "stop-print"]:
                    ctrls["custom_cmd"].append(cmd)
                return None

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
                    self.printerController[printer].send_proj_cmd(cmd)
                    time.sleep(.5)
        if len(ctrls["pi_cmds"]) != 0:
            # Send commands to pi controller logic
            for cmd in ctrls["pi_cmds"]:
                # Find out who is the message for
                dest = cmd.split("-")[-1]
                printers = self.get_printers_num(dest)
                for printer in printers:
                    self.printerController[printer].send_pi_cmd(cmd)

        # if len(ctrls["custom_cmd"]) != 0:
        #     # Send custom command to all systems
        #     # Find out who is the message for
        #     # dest = cmd.split("-")[-1]
        #     # printers = self.get_printers_num(dest)
        #     for cmd in ctrls["custom_cmd"]:
        #         if cmd == "stop-run":
        #             self.send_stop_run_cmd()
        #         elif cmd == "start-run":
        #             self.start_run_cmd()
        #         elif cmd == "continue-run":
        #             self.send_continue_run_cmd()
        #         elif cmd == "start-print":
        #             self.start_print_cmd()
        #         elif cmd == "stop-print":
        #             self.stop_print_cmd()
        #         elif cmd == "get-videos":
        #             self.get_logger().warning("get videos is herererere")
        #             req = Video.Request()
        #             req.cmd = "get-videos"
        #             self.printerController[0].client_req("pi", req)

        ###################################################################################
        self.get_logger().debug('Finished Dispatching Gui Commands...\n')    # DEBUG MESSGE
        ###################################################################################
        return 0

    def send_start_run_cmd(self):
        # self.get_logger().info("[SEND_START_RUN()]: Started")
        # self.okToRunEvent.set()
        # startRunThread = threading.Thread(target=self.start_run())

        # # Allow the thread to run in the backgorund
        # startRunThread.daemon = True
        # startRunThread.start()
        # self.get_logger().info("[SEND_START_RUN_CMD()]: Ended")
        pass

    # def send_stop_run_cmd(self):
    #     # self.okToRunEvent.clear()
    #     # # self.stopAllEvent.set()
    #     # self.turn_off_all_systems()
    #     pass

    # def send_continue_run_cmd(self):
    #     # # Move leve to the next leve
    #     # print(self.levelController._curr_level)
    #     # # self.stopAllEvent.clear()
    #     # printers = self.get_printers_num("all")
    #     # # if self.stopAllEvent.is_set():
    #     # #     return 1
    #     # next_level = (self.levelController._curr_level + 1) % 5
    #     # self.get_logger().info(
    #     #     "[send_continue_run_cmd()]: sending to level %d\n" % (next_level))
    #     # t = threading.Thread(target=self.levelController.send_level_cmd, args=[
    #     #                      "level-motors-" + str(next_level)])
    #     # t.start()
    #     # t.join()
    #     # while t.is_alive():
    #     #     time.sleep(0.2)

    #     # # if self.stopAllEvent.is_set():
    #     # #     return 1
    #     # self.get_logger().info("[send_continue_run_cmd()]: playing videos")
    #     # for printer in printers:
    #     #     self.printerController[printer].send_pi_cmd("pi-play-queue-" + testVar)
    #     #     self.printerController[printer].send_motor_cmd("motor-on-9-" + testVar)
    #     # # if self.stopAllEvent.is_set():
    #     # #     for printer in printers:
    #     # #         self.printerController[printer].send_proj_cmd(
    #     # #             "proj-led-off-" + testVar)
    #     # #         self.printerController[printer].send_pi_cmd(
    #     # #             "pi-stop-queue-" + testVar)
    #     # #         self.printerController[printer].send_motor_cmd("motor-off-" + testVar)
    #     # #     return 1
    #     # self.get_logger().info("[send_continue_run_cmd()]: Finished")
    #     pass

    # def start_print_cmd(self):
    #     self.get_logger().info("[SEND_START_PRINT_CMD()]: Started")
    #     # # self.stopAllEvent.clear()
    #     self.start_print()
    #     # self.ledOffThread = threading.Thread(target=self.start_print)
    #     # self.ledOffThread.daemon = True
    #     # self.ledOffThread.start()
    #     pass
    
    # def start_print(self):
    #     self.get_logger().info("[START_PRINT]: Started")
    #     # self.decode_gui_cmd("proj-led-on-" + testVar)
    #     self.printerController[0].send_proj_cmd("proj-led-on-" + testVar)
    #     self.get_logger().info("Turned leds on")
    #     # time.sleep(20)
    #     # self.decode_gui_cmd("proj-led-off-" + testVar)
    #     # self.decode_gui_cmd("pi-stop-queue-" + testVar)
    #     self.get_logger().info("[START_PRINT]: Ended")

    # def stop_print_cmd(self):
    #     # if self.ledOffThread != None:
    #     #     self.ledOffThread.cancel()
    #     self.stop_print()

        
        
    # def stop_print(self):
    #     self.decode_gui_cmd("proj-led-off-" + testVar)
    #     self.decode_gui_cmd("pi-stop-queue-" + testVar)

    # def start_run_cmd(self):
    #     # turns off all systems
    #     # resets playing queue
    #     # homes the level controller
    #     # moves to level 0
    #     # starts rotating the motors
    #     self.get_logger().info("[START_RUN_CMD()]: Started")
    #     # self.stopAllEvent.clear()
    #     self.decode_gui_cmd("proj-led-off-" + testVar)
    #     # self.decode_gui_cmd("pi-stop-video-" + testVar)
    #     self.decode_gui_cmd("motor-off-" + testVar)
    #     # self.decode_gui_cmd("pi-reset-queue-" + testVar)
        
    #     self.get_logger().info("[START_RUN_CMD()]: sending home")
    #     # Home the motors
    #     self.levelController.send_level_cmd("level-motors-home")
    #     # wait before sending the next message
    #     time.sleep(2)
    #     # Make sure we havent cancel the next commnad
    #     # if self.stopAllEvent.is_set():
    #     #     return 1
    #     self.get_logger().info("[START_RUN_CMD()]: Sending level 0")
    #     # Send the motors to the first leve
    #     self.levelController.send_level_cmd("level-motors-0")
    #     # Wait before sending next command
    #     time.sleep(1)
    #     # Verify we can still run
    #     # if self.stopAllEvent.is_set():
    #     #     return 1
    #     # Turn on motors
    #     self.decode_gui_cmd("motor-on-9-" + testVar)
    #     self.get_logger().info("[START_RUN_CMD()]: Ended")

    # def start_print(self):

    #     pass

    ################################################ Client Request ##############################################

    # TODO: make sure to remember to update the project and motor classes from the response data

    def get_printers_num(self, dest):
        if dest == "all":
            return [0, 1, 2, 3, 4]
        return [int(dest)]

    # def turn_off_all_systems(self):
    #     # if self.levelController._is_moving:
    #     #     self.levelController.goal_handle.cancel_goal_async()
    #     self.decode_gui_cmd("proj-led-off-" + testVar)
    #     self.decode_gui_cmd("pi-stop-video-" + testVar)
    #     self.decode_gui_cmd("motor-off-" + testVar)


def main(args=None):
    rclpy.init(args=args)
    #print('In Main thread with thread_id: %d\n' % threading.get_native_id())
    service = MainLogicNode()
    # service.send_height_goal(10)

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
