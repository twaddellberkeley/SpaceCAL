

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
        self.srv = self.create_service(GuiInput, 'gui_input_srv', self.main_logic_callback)

        #***** Clients *******#
        self.proj_cli = self.create_client()
        self.motor_cli = self.create_client()
        self.video_cli = self.create_client()


    def main_logic_callback(self, request, response):
        self.get_logger().info('Incoming request\ncmd: %s ' % (request.cmd))

        # parse and decode the command
        cmd_list = self.cmd_decoder(request.cmd)

        # send the commads to the dispacher
        res = self.dispatcher(cmd_list)
        # dispatch the proper controller to its own thread
        # there are two controllers --> plataform controller
        #                           --> Projection controller
        # projection controller has sub controllers --> motor controllers
        #                                           --> projector controller
        #                                           --> video controller
        # dispatch sub controllers on different threads
        
        response.err = 0
        return response

    def cmd_decoder(self, cmd):
        # this function should do all the dirty work of converting a string command into 
        # an array of commnads for the valid controller/controllers.
        cmds = []
        self.get_logger().info('Decoding command:  %s\n' % cmd)
        
        cmd_list = cmd.split("-")
        for c in cmd_list:
            if c == "gui":
                self.get_logger().info('I come from gui\n')
            elif c == "main":
                self.get_logger().info('I come from main\n')
            else:
                self.get_logger().info('This is my command: %s\n' % c)
        return cmd_list
        
    def dispatcher(self, lst):
        # It reads from a list of commands and dispatches them in their own thread
        self.get_logger().info('Dispatching commands...\n')




def main(args=None):
    rclpy.init(args=args)

    minimal_service = MainLogicNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()