
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

        #***** Initialize printer objects ******#
        self._printer = [Printer(0), Printer(1), Printer(2), Printer(3), Printer(4)]



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
        assert(proj_cmd != None)
        split_cmd = proj_cmd.split("-")
        if split_cmd[0] == "on":
            # Turn projector on
            self.proj_client_req(proj_cmd)
        elif split_cmd[0] == "off":
            # Trun projector off
            if self.is_proj_led_on(split_cmd[1]):
                # If the led is off turn it off first
                self.proj_client_req("led-off-" + split_cmd[1])
            self.proj_client_req(proj_cmd)
        elif split_cmd[0] == "led":
            if split_cmd[1] == "off":
                # Turn led off
                self.proj_client_req(proj_cmd)
            elif split_cmd[1] == "on":
                # Turn led on
                if not self.is_proj_on(split_cmd[2]):
                    # Verify projector is on else turn it on first
                    self.proj_client_req("on-" + split_cmd[2])
                self.proj_client_req(proj_cmd)
            else:
                self.get_logger().error('Command not recognized in project logic: ')
        else:
            self.get_logger().error('Command not recognized in project logic: ')

    def pi_controller_logic(self, pi_cmd):
        pass

    def proj_client_req(self, cmd):
        '''This Function send the commands to the projectors mantioned in the command itself'''
        if cmd.split("-")[-1] == "all":
            self.proj_cli = [None] * 5
            for i in range(5):
                #***** Clients *******#
                self.proj_cli[i] = self.create_client(Projector, 'projector_srv_'+str(i))
                while not self.proj_cli[i].wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')

                # Populate request
                request = Projector.Request()
                request.cmd = cmd[0:len(cmd)-len("all")] + str(i)
                self.future = self.proj_cli[i].call_async(request)
                self.future.add_done_callback(partial(self.proj_future_callback))

                self.get_logger().info('Waiting on async...')  # DEBUG message
               
        elif cmd.split("-")[-1] in ['0','1','2','3','4']:
            #***** Clients *******#
            i = int(cmd.split("-")[-1])
            self.proj_cli = self.create_client(Projector, 'projector_srv_' + cmd.split("-")[-1])
            while not self.proj_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            # Populate request
            request = Projector.Request()
            request.cmd = cmd
            self.future = self.proj_cli.call_async(request)
            self.future.add_done_callback(partial(self.proj_future_callback))

            self.get_logger().info('Waiting on async...')   # DEBUG message
            
        else:
            self.get_logger().error('No command: %s\n' %(request.cmd))


    def proj_future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('status %s' % (response.status))
            if response.err == 0:
                self._printer[0]._isLedOn = response.is_led_on
                self._printer[0]._isVideoOn = response.is_video_on
                self.get_logger().info('response.is_led_on %s' % ("True" if response.is_led_on else "False"))
                

        except Exception as e:
            self.get_logger().error('ERROR: --- %r' %(e,))
        self.get_logger().info('finished async call....')
        self.get_logger().info('Printer_1._isLedOn %s' % ("True" if self._printer[0]._isLedOn else "False"))
    
    #********* Utility functions ***********#
    def is_proj_on(self, proj_num):
        assert type(proj_num) == type("")
        if proj_num == "all":
            for p in range(5):
                if not self._printer[p]._isLedOn:
                    return False
            return True
        else:
            return self._printer[int(proj_num)]._isLedOn



def main(args=None):
    rclpy.init(args=args)
    #print('In Main thread with thread_id: %d\n' % threading.get_native_id())
    minimal_service = MainLogicNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
