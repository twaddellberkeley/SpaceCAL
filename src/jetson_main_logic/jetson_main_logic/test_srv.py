from interfaces.srv import MotorSrv, ProjectorSrv

import time
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.motor_srv = self.create_service(
            MotorSrv, 'motor_command_1', self.print_command)

    def print_command(self, request, response):
        self.get_logger().info('Incoming request\na: %d b: %d' %
                               (request.cmd_num, request.value))
        response.ok = True
        response.error = 10
        time.sleep(3)
        self.get_logger().info('After 3 seconds \na: %d b: %d' %
                               (response.ok, response.error))
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
