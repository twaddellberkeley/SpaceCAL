
import time
import threading
import rclpy
from rclpy.node import Node
from smbus2 import SMBus

from interfaces.srv import MotorSrv

from .submodules.motor_controller import Motor

# Maximum rotational speed for vile in rot/min
MAX_MAG_SPPED = 20

class MotorNode(Node):
    MAX_SPEED = MAX_MAG_SPPED
    MIN_SPEED = -MAX_MAG_SPPED

    def __init__(self):
        super().__init__('print_motor_node')
        # Declare a parameter that can be set from a launch file to create distinct services: One for each Motor.
        # The default value of the parameter is zero and address 18
        ###############################################################################################################
        self.declare_parameter("motor_number", 0)
        self.declare_parameter("address", 18)
        self.motor_num = self.get_parameter(
            'motor_number').value
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
        self.motor.resetMotor()

        # Create a thread to keep the motors energyzed in ordered to mantain its position
        # TODO: This line may need to be deleted
        stayAlive = threading.Thread(target=self.motor.stay_alive)

        # Allow the thread to run in the backgorund
        stayAlive.daemon = True
        stayAlive.start()

        # Energize the motor
        self.motor.exit_safe_start()
        self.motor.energize()

    def print_motor_exec_callback(self, request, response):
        assert type(request.cmd) == type(""), "cmd is not a string"
        self.get_logger().info('\nService request recieved at Print Motor %d\nCommand: %s ' %
                               (self.motor_num, request.cmd))

        # prosses the commands
        res, speed = self.process_cmds(request)

        response.err = res
        response.msg = "Print Motor " + \
            str(self.motor_num) + " executed succesfully"
        
        
        response.set_speed = speed
        if speed != 0:
            response.status = "Rotating"
        else:
            response.status = "Off"
        self.get_logger().info('Finished request from Motor\ncmd: %s ' % (request.cmd))
        return response

    def process_cmds(self, request):
        cmd = request.cmd
        speed = request.speed
        assert type(cmd) == type(""), "command must be a string type"
        assert type(speed) == type(9), "Speed requested must be of type int"
        self.get_logger().info('Decoding Gui Command:  %s\n' % cmd)

        # Verify speed Max and Min
        if speed < self.MIN_SPEED:
            speed = self.MIN_SPEED
        elif speed > self.MAX_SPEED:
            speed = self.MAX_SPEED
        
        if speed == 0:
            self.motor.powerdown()
            return 0, speed
        elif self.motor.getCurrentStatus() != 10:
            self.motor.exit_safe_start()
            self.motor.energize()
            status = self.motor.getCurrentStatus()
            time.sleep(0.1)
            if self.motor.getCurrentStatus() != 10:
                time.sleep(0.1)
                if self.motor.getCurrentStatus() != 10:
                    self.get_logger().error('Could not energize motor:  %d\n' % (self.motor.id))
                    return -1
            
        # Setting target speed
        self.motor.setTargetSpeed(speed)
        count = 0
        while abs(self.motor.getCurrentVelocity() - speed) > 1:
            self.get_logger().info("Moving... Current speed: %d" %(self.motor.getCurrentVelocity()))
            if count > 10:
                self.get_logger().warning("Could not achieved desired speed: %d" %(speed))
                break
            count += 1
            time.sleep(0.5)
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
