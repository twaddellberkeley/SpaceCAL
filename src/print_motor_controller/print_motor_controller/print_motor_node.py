

import time
import threading
import rclpy
from rclpy.node import Node
from smbus2 import SMBus, i2c_msg

from interfaces.srv import MotorSrv

from .submodules.motor_controller import TicI2C

class MotorNode(Node):
    def __init__(self):
        super().__init__('print_motor_node_0')
        self.declare_parameter("Address")

        adrNum = self.get_parameter('Address').get_parameter_value()
        self.address = adrNum.integer_value
        self.proj_srv = self.create_service(MotorSrv, 'print_motor_srv_0', self.print_motor_exec_callback)

        bus = SMBus(busNum=1) # TODO: figureout how to get the bus number automatically
        global tic
        tic = TicI2C(bus, address)
        tic.reset_motor()
        time.sleep(0.1)
        # Keep telling motor that were connected
        stayAlive = threading.Thread(target=tic.stay_alive)
        stayAlive.daemon = True
        stayAlive.start()
        tic.exit_safe_start()
        tic.energize()
        # Keep publishing current status of motor
        publishData = threading.Thread(target=self.publish_data, args=())
        publishData.daemon = True
        publishData.start()

       

    def print_motor_exec_callback(self, request, response):
        assert type(request.cmd) == type(""), "cmd is not a string"
        self.get_logger().info('Incoming request\ncmd: %s ' % (request.cmd))
        time.sleep(5)
        response.err = 0
        response.msg = "Motor 0 executed succesfully"
        response.status = request.cmd.split("-")[-1]
        response.is_video_on = True
        response.is_led_on = True
        self.get_logger().info('Finished request from Motor\ncmd: %s ' % (request.cmd))
        return response


def main(args=None):
    print('Hi from print_motor_controller 0.')
    rclpy.init(args=args)

    service = MotorNode()

    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
