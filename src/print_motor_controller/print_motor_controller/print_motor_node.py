

from decimal import MIN_EMIN
import time
import threading
import rclpy
from rclpy.node import Node
from smbus2 import SMBus

from interfaces.srv import MotorSrv

from .submodules.motor_controller import Motor


class MotorNode(Node):
    MAX_SPEED = 40
    MIN_SPEED = -40

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

        self.motor.on()

    # TODO: Write the logic of what to do when a request is reviced

    def print_motor_exec_callback(self, request, response):
        assert type(request.cmd) == type(""), "cmd is not a string"
        self.get_logger().info('\nService request recieved at Print Motor %d\nCommand: %s ' %
                               (self.motor_num, request.cmd))

        # dispach the commands
        res, speed = self.process_cmds(request.cmd)

        response.err = res
        response.msg = "Print Motor " + \
            str(self.motor_num) + " executed succesfully"
        response.status = "NORMAL"
        response.set_speed = speed
        self.get_logger().info('Finished request from Motor\ncmd: %s ' % (request.cmd))
        return response

    def process_cmds(self, raw_cmd):
        assert type(raw_cmd) == type(""), "command must be a string type"

        self.get_logger().info('Decoding Gui Command:  %s\n' % raw_cmd)

        # where the actual command starts this may change later
        index = 0

        speed = 0

        # Split comnands
        cmd_list = raw_cmd.split("_")
        for cmd in cmd_list:
            split_cmd = cmd.split("-")
            if "on" == split_cmd[index]:
                self.get_logger().info('Motor ON:  ')
                # Get speed
                speed = int(split_cmd[index + 1])

            elif "off" == split_cmd[index]:
                self.get_logger().info('Motor Off:  ')
                # Get speed
                speed = 0

            # Verify speed Max and Min
            if speed < self.MIN_SPEED:
                speed = self.MIN_SPEED
            elif speed > self.MAX_SPEED:
                speed = self.MAX_SPEED

            # Setting target speed
            self.motor.set_speed(speed)

        return 0, speed


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
