
from threading import Thread
import threading
from xml.etree.ElementTree import tostring
from functools import partial

from interfaces.srv import GuiDisplay, GuiInput, Projector, Video

import rclpy
from rclpy.node import Node

commands = ["proj-on-all", "proj-off-all", "rotate-vile-30"]

class Motor():
    
    def __init__(self, id):
        super().__init__()
        self._id = id
        self._status = "off"
        self._speed = 0

class Printer():
    def __init__(self, id):
        super().__init__()
        self._id = id
        self._status = "off"
        self._isLedOn = False
        self._isVideoOn = False
        self._motor = Motor(id)

class PrintController(Node):
    # This class will controll the logic for the printer which involves three sub
    # controllers. (1) Motor controller which controls a single motor. (2) Projector controller
    # which controls the the state of the LED projector (no/off). (3) Video Controller which 
    # controls the video being projected. 
    def __init__(self):
        super().__init__("PrinterControllerNode")
        self._printer_1 = Printer(1)
        self._printer_1 = Printer(2)
        self._printer_1 = Printer(3)
        self._printer_1 = Printer(4)
        self._printer_1 = Printer(5)
    
    def send_cmd(self, cmd):
        assert(len(cmd) > 0)
        cmds = self.cmd_decoder(cmd)
        self.dispatch(cmds)

    def cmd_decoder(self, cmds):
        self.get_logger().info('Decoding Printer Command:  \n' )
        assert(len(cmds) > 0)
        print_cmds = {"proj_cmds":[], "pi_cmds":[], "motor_cmds":[]}
        for cmd in cmds:
            if "proj" in cmd:
                self.get_logger().info('Appending command in Printer...:  \n' )
                print_cmds["proj_cmds"].append(cmd)
            elif "pi" in cmd:
                print_cmds["pi_cmds"].append(cmd)
            elif "motor" in cmd:
                print_cmds["motor_cmds"].append(cmd)
            else:
                self.get_logger().error('Error: ---- invalid command in Printer Controller\n')
        return print_cmds
    
    def dispatch(self, print_cmds):
        self.get_logger().info('Dispatching Printer Command:  \n' )

        if len(print_cmds["motor_cmds"]) > 0:
            # lets process this first 
            pass
        
        if len(print_cmds["proj_cmds"]) > 0:
            # lets process this second
            for p_cmd in print_cmds["proj_cmds"]:
                cmd = p_cmd.split("-")
                if cmd[1] == "on":
                    # Turn on projector
                    self.proj_controller(p_cmd)
                elif cmd[1] == "off":
                    # Turn off projector (veryfy led is off first)
                    pass
                elif cmd[1] == "led":
                    # is an led commad
                    if cmd[2] == "on":
                        # Turn led on (make sure projedtor is on)
                        pass
                    elif cmd[2] == "off":
                        # Turn led off
                        pass
            
        if len(print_cmds["pi_cmds"]) > 0:
            # process this commands last
            pass

   


class LevelController():
    # This class controls the level of the paylod's plataform. 
    def __init__(self):
        super().__init__()


    def send_cmd(self, cmd):
            pass

    
    def cmd_decoder(self):
        self.get_logger().info('Decoding Printer Command:  \n' )
        


class MainLogicNode(Node):

    def __init__(self):
        super().__init__('main_logic_node')

        ################## Services ####################
        #***** Servers ******#
        self.gui_srv = self.create_service(GuiInput, 'gui_input_srv', self.gui_input_callback)

        #***** Clients *******#
        # self.proj_cli = self.create_client(Projector, 'projector_srv')
        self.gui_cli = self.create_client(GuiDisplay, 'gui_display_srv')
        self.video_cli = self.create_client(Video, 'video_srv')

        #***** Initialize Controllers *****#
        self.printer_controller = PrintController()
        self.level_controller = LevelController()

        #***** Initialize printer objects ******#
        self._printer_1 = Printer(1)
        self._printer_2 = Printer(2)
        self._printer_3 = Printer(3)
        self._printer_4 = Printer(4)
        self._printer_5 = Printer(5)


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
        ctrls = {"level_cmds":[], "motor_cmds":[], "proj_cmds":[], "pi_cmds":[]}
        # Split comnands
        cmd_list = raw_cmd.split("_")
        for cmd in cmd_list:
            split_cmd = cmd.split("-")
            if "proj" == split_cmd[0]:
                # extract project command
                ctrls["proj_cmds"].append(cmd[len("proj-"):len(cmd)])
                self.get_logger().info('******** first cmd in proj_cmds: %s\n' % ctrls["proj_cmds"][0])
            elif "pi" == split_cmd[0]:
                # extract pi command 
                ctrls["pi_cmds"].append(cmd[len("pi-"):len(cmd)])
                self.get_logger().info('******** first cmd in pi_cmds: %s\n' % ctrls["pi_cmds"][0])
            elif "motor" == split_cmd[0]:
                # extract motor command
                ctrls["motor_cmds"].append(cmd[len("motor-"):len(cmd)])
                self.get_logger().info("******** first cmd in motor_cmds: %s\n" % ctrls["motor_cmds"][0])
            elif "level" == split_cmd[0]:
                # extract level commands 
                ctrls["level_cmds"].append(cmd[len("level-"):len(cmd)])
                self.get_logger().info('******** first cmd in level_cmds: %s' % ctrls["level_cmds"][0])     
        return ctrls
        
    def dispatch_gui_cmd(self, ctrls):
        # It reads from a list of commands and dispatches them in their own thread
        self.get_logger().info('Dispatching Gui Commands...\n')

        
        if len(ctrls["level_cmds"]) != 0:
            # Send command to level controller logic
            for cmd in ctrls["level_cmds"]:
                self.level_controller_logic(cmd)
        
        if len(ctrls["motor_cmds"]) != 0:
            # Send commadns to motor controller logic
            for cmd in ctrls["motor_cmds"]:
                self.motor_controller_logic(cmd)
            
        if len(ctrls["proj_cmds"]) != 0:
            # Send commands to projector controller logic
            for cmd in ctrls["proj_cmds"]:
                self.proj_controller_logic(cmd)

        if len(ctrls["pi_cmds"]) != 0:
            # Send commands to pi controller logic
            for cmd in ctrls["pi_cmds"]:
                self.pi_controller_logic(cmd)

        self.get_logger().info('Finished Dispatching Gui Commands...\n')
        return 0

    def level_controller_logic(self, level_cmd):
        pass

    def motor_controller_logic(self, motor_cmd):
        pass

    def proj_controller_logic(self, proj_cmd):
        pass

    def pi_controller_logic(self, pi_cmd):
        pass

    def proj_controller(self, cmd):
         #***** Clients *******#
        self.proj_cli = self.create_client(Projector, 'projector_srv')
        while not self.proj_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        #self.get_logger().info('In thread proj_controller with thread_id: %d\n' % threading.get_native_id())
        request = Projector.Request()
        request.cmd = cmd
        self.future = self.proj_cli.call_async(request)
        self.future.add_done_callback(partial(self.proj_callback))

        self.get_logger().info('Waiting on async...')
        # rclpy.spin_until_future_complete(self, self.future)
        
    
    def proj_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('ERROR: --- %r' %(e,))
        self.get_logger().info('finished async call....')
    

def main(args=None):
    rclpy.init(args=args)
    #print('In Main thread with thread_id: %d\n' % threading.get_native_id())
    minimal_service = MainLogicNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
