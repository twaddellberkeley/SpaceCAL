
from threading import Thread
import threading

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
        self.proj_cli = self.create_client(Projector, 'projector_srv')
        self.gui_cli = self.create_client(GuiDisplay, 'gui_display_srv')
        self.video_cli = self.create_client(Video, 'video_srv')


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
            if c == "main":
                self.get_logger().info('I come from gui\n')
                cmds.append((c, "cmd"))
            elif c == "projector":
                self.get_logger().info('I come from main\n')
                cmds.append((c,"cmd"))
            else:
                self.get_logger().info('This is my command: %s\n' % c)
        return cmds
        
    def dispatcher(self, lst):
        # It reads from a list of commands and dispatches them in their own thread
        self.get_logger().info('Dispatching commands...\n')
        self.proj_controller(lst[0][1])

        self.get_logger().info('Dispatched command!!!\n')
        # t = []
        # i = 0
        # for c in lst:
        #     if c[0] == "main":
        #         th = Thread(targets=self.controller, agrs=(c[1],))
        #         t.append(th)
        #         i+=1
        #     elif c[0] == "projector":
        #         self.get_logger().info('Creating projector Tread...\n')
        #         th = Thread(target=self.proj_controller, args=(c[1],))
        #         t.append(th)
        #         i+=1

        # for i in range(len(t)):
        #     t[i].start()

        # for i in range(len(t)):
        #     t[i].join()

        self.get_logger().info('Finished request...\n')
        return 0


    def controller(self, request):
        while not self.gui_clit.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.future = self.gui_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def proj_controller(self, request):
        while not self.proj_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        #self.get_logger().info('In thread proj_controller with thread_id: %d\n' % threading.get_native_id())
        req = Projector.Request()
        req.cmd = request
        self.future = self.proj_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    #print('In Main thread with thread_id: %d\n' % threading.get_native_id())
    minimal_service = MainLogicNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()