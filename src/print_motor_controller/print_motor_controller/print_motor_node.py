

import time
import threading
import rclpy
from rclpy.node import Node
from smbus2 import SMBus

from interfaces.srv import MotorSrv

from .submodules.motor_controller import Motor


class MotorNode(Node):
    def __init__(self):
        super().__init__('print_motor_node')
        # Declare a parameter that can be set from a launch file to create distinct services: One for each Motor.
        # The default value of the parameter is zero and address 18
        ###############################################################################################################
        self.declare_parameter("motor_number", 0)
        # ROS Parameters: "motor_number"  ######
        self.declare_parameter("address", 18)
        self.motor_num = self.get_parameter(
            'motor_number').value  # "address"      ######
        self.address = self.get_parameter("address").value
        ###############################################################################################################

        # Create service to recieve motor request
        self.proj_srv = self.create_service(
            MotorSrv, 'print_motor_srv_' + str(self.motor_num), self.print_motor_exec_callback)

        # Create a bus object with with motor on /dev/i2c-1
        # TODO: figureout how to get the bus number automatically
        bus = SMBus(1)

        # Create a motor object that controls the motor and holds its state.
        self.motor = Motor(self.motor_num, bus,
                           self.address, self.get_logger())
        self.motor.reset()

        # Create a thread to keep the motors energyzed in ordered to mantain its position
        # TODO: This line may need to be deleted
        stayAlive = threading.Thread(target=self.motor.stay_alive)

        # Allow the thread to run in the backgorund
        stayAlive.daemon = True
        stayAlive.start()

        self.motor.exit_safe_start()
        self.motor.energize()

    # TODO: Write the logic of what to do when a request is reviced

    def print_motor_exec_callback(self, request, response):
        assert type(request.cmd) == type(""), "cmd is not a string"
        self.get_logger().info('\nService request recieved at Print Motor %d\nCommand: %s ' %
                               (self.motor_num, request.cmd))
        time.sleep(5)
        response.err = 0
        response.msg = "Print Motor " + \
            str(self.motor_num) + " executed succesfully"
        response.status = request.cmd.split("-")[-1]
        response.is_video_on = True
        response.is_led_on = True
        self.get_logger().info('Finished request from Motor\ncmd: %s ' % (request.cmd))
        return response


def main(args=None):

    rclpy.init(args=args)
    service = MotorNode()
    print('Hi from print_motor_controller %d.', service.motor_num)
    try:
        rclpy.spin(service)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        # Stopping motor completely
        service.motor.powerdown()
        service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
