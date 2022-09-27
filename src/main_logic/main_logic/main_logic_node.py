
from threading import Thread
import threading
from xml.etree.ElementTree import tostring
from functools import partial

from interfaces.srv import GuiDisplay, GuiInput, Projector, Video

import rclpy
from rclpy.node import Node

class PrintController():
    # This class will controll the logic for the printer which involves three sub
    # controllers. (1) Motor controller which controls a single motor. (2) Projector controller
    # which controls the the state of the LED projector (no/off). (3) Video Controller which 
    # controls the video being projected. 
    pass

class LevelController():
    # This class controls the level of the paylod's plataform. 
    pass


class MainLogicNode(Node):

    def __init__(self):
        super().__init__('main_logic_node')

        ################## Services ####################
        #***** Servers ******#
        self.gui_srv = self.create_service(GuiInput, 'gui_input_srv', self.main_logic_callback)

        #***** Clients *******#
        # self.proj_cli = self.create_client(Projector, 'projector_srv')
        self.gui_cli = self.create_client(GuiDisplay, 'gui_display_srv')
        self.video_cli = self.create_client(Video, 'video_srv')


    def main_logic_callback(self, request, response):
        self.get_logger().info('Incoming request\ncmd: %s ' % (request.cmd))

        # Record metadata and save

        # parse and decode the command
        cmd_list = self.cmd_decoder(request.cmd)

        # dispach the commands
        res = self.dispatcher(cmd_list)
        # dispatch the proper controller to its own thread
        # there are two controllers --> plataform controller
        #                           --> Projection controller
        # projection controller has sub controllers --> motor controllers
        #                                           --> projector controller
        #                                           --> video controller
        # dispatch sub controllers on different threads
        
        response.err = res
        return response

    def cmd_decoder(self, cmd):
        # this function should do all the dirty work of converting a string command into 
        # an array of commnads for the valid controller/controllers.
        
        
        # Projector and Level Commands
        cmds = {"proj_cmds":[], "level_cmd":[]}
        
        self.get_logger().info('Decoding command:  %s\n' % cmd)
        
        # Projector and Level Commands
        cmds = {"proj_cmds":[], "level_cmd":[]}
        
        
        cmd_list = raw_cmd.split("-")

        for cmd in cmd_list:
            match cmd:
                case commands["projector-on"]:
                    # define command to turn projector on
                    cmds["proj_cmds"].append("led_on")
                case _:
                    print("Nothing here")
               

        return cmds
        
    def dispatcher(self, cmds):
        # It reads from a list of commands and dispatches them in their own thread
        self.get_logger().info('Dispatching commands...\n')

        for cmd in cmds:
            if "projector" in cmd:
                self.proj_controller(cmd)
        # self.get_logger().info('Request response: %d\n' % res)

        self.get_logger().info('Finished dispatching...\n')
        return 0


    def controller(self, request):
        while not self.gui_clit.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.future = self.gui_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

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
