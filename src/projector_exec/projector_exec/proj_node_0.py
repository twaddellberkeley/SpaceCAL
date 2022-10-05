
import time
from interfaces.srv import Projector

import rclpy
from rclpy.node import Node

class ProjectorNode(Node):
    

    def __init__(self):
        super().__init__('projector_node_0')
        self.proj_srv = self.create_service(Projector, 'projector_srv_0', self.projector_exec_callback)
       

    def projector_exec_callback(self, request, response):
        assert type(request.cmd) == type(""), "cmd is not a string"
        self.get_logger().info('Incoming request\ncmd: %s ' % (request.cmd))
        time.sleep(5)
        response.err = 0
        response.msg = "Projector 0 executed succesfully"
        response.status = request.cmd.split("-")[-1]
        response.is_video_on = True
        response.is_led_on = True
        self.get_logger().info('Finished request from Projector\ncmd: %s ' % (request.cmd))
        return response


def main(args=None):
    print('Hi from projector_exec 0.')
    rclpy.init(args=args)

    minimal_service = ProjectorNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()



if __name__ == '__main__':
    main()
