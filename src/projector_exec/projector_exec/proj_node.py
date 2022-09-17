
import time
from interfaces.srv import Projector

import rclpy
from rclpy.node import Node

class ProjectorNode(Node):

    def __init__(self):
        super().__init__('projector_node')
        self.proj_srv = self.create_service(Projector, 'projector_srv', self.projector_exec_callback)

    def projector_exec_callback(self, request, response):
        self.get_logger().info('Incoming request\ncmd: %s ' % (request.cmd))
        time.sleep(2)
        response.err = 0
        return response


def main(args=None):
    print('Hi from projector_exec.')
    rclpy.init(args=args)

    minimal_service = ProjectorNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()



if __name__ == '__main__':
    main()
