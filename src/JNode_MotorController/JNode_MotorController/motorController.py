# Copyright 2021 UC-Berkeley
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


# Author: Taylor Waddell
# Link to tic: pololu.com/docs/0J71/12.9
# Uses the smbus2 library to send and receive data from a Tic.
# Works on Linux with either Python 2 or Python 3.
#
# NOTE: The Tic's control mode must be "Serial / I2C / USB".
# NOTE: For reliable operation on a Raspberry Pi, enable the i2c-gpio
#   overlay and use the I2C device it provides (usually /dev/i2c-3).
# NOTE: You might nee to change the 'SMBus(3)' line below to specify the
#   correct I2C device.
# NOTE: You might need to change the 'address = 11' line below to match
#   the device number of your Tic.
import rclpy
import time
import threading
from rclpy.node import Node
from std_msgs.msg import String, Int32
from smbus2 import SMBus, i2c_msg
# globl vars
# Tic object
tic = None
# Pulse to move moto
# incrBit = 51200 # Is one rotation or one inch
incrBit = 51200/25.4 # Is 1mm per bit unit

# Step veloicty to achieve 1rot/min
velBit = 1024000000/60
# Logger of the node
logger = None
# Address and bus of motor controlling
busNum = 1
address = 14


# Tic class with generic motor controls
class TicI2C(object):
    # Init function, takes bus and address
    def __init__(self, bus, address):
        # Bus object and address object passed down
        self.bus = bus
        self.address = address
        # See if we can read a bit to test if motor is open
        try:
            self.bus.read_byte_data(self.address, 0)
        except Exception:
            logger.error(
                "Could not open motor controller on address %d" % self.address)
            exit(0)
    # Sends the "Exit safe start" command.

    def exit_safe_start(self):
        command = [0x83]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Sets the target position.
    #
    # For more information about what this command does, see the
    # "Set target position" command in the "Command reference" section of the
    # Tic user's guide.
    # Size per 1/256 microstep is .0004960938mm in linear length
    def set_target_position(self, target):
        logger.info("Target %d" % target)
        command = [0xE0,
                   target >> 0 & 0xFF,
                   target >> 8 & 0xFF,
                   target >> 16 & 0xFF,
                   target >> 24 & 0xFF]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Sets target speed
    def set_target_speed(self, speed):
        command = [0xE3,
                   speed >> 0 & 0xFF,
                   speed >> 8 & 0xFF,
                   speed >> 16 & 0xFF,
                   speed >> 24 & 0xFF]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Tells motor to home
    def go_home(self):
        command = [0x97,0]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Tells motor to de-energize
    def powerdown(self):
        command = [0x86]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Tells motor to energize
    def energize(self):
        command = [0x85]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Tells Motor to halt
    def halt_motor(self):
        command = [0x89]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Reset
    def reset_motor(self):
        command = [0xB0]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Cleare timeout, ensure motor runs
    def clear_Timeout(self):
        command = [0x8C]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Stay alive, keep motor running
    def stay_alive(self):
        while rclpy.ok():
            try:
                self.clear_Timeout()
            except Exception:
                logger.warn("MOTOR DISCONNECTED %d" % self.address)
                logger.warn("SHUTTING DOWN NODE")
                rclpy.shutdown()
                exit(0)
        time.sleep(.5)

    # Gets one or more variables from the Tic.
    def get_variables(self, offset, length):
        write = i2c_msg.write(self.address, [0xA1, offset])
        read = i2c_msg.read(self.address, length)
        self.bus.i2c_rdwr(write, read)
        return list(read)
        logger.info(str(busNum))

    # Gets the "Current position" variable from the Tic.
    def get_current_position(self):
        b = self.get_variables(0x22, 4)
        position = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24)
        if position >= (1 << 31):
            position -= (1 << 32)
        return position

    # Gets the "Current speed" variable from the Tic.
    def get_current_velocity(self):
        b = self.get_variables(0x26, 4)
        velocity = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24)
        if velcocity >= (1 << 31):
            velocity -= (1 << 32)
        return velocity

    # Gets current status
    def get_current_status(self):
        b = self.get_variables(0x00, 1)
        status = b[0]
        return status

    # Gets current flag
    def get_current_flags(self):
        b = self.get_variables(0x01, 1)
        status = b[0]
        return status


class keySubscriber(Node):
    def __init__(self):
        # Create the subscriber node that listens to key input
        super().__init__('MotorControllerSubscriber')
        self.subscriptionInput = self.create_subscription(
            String,
            'keyinput',
            self.checkRun,
            10)
        self.subscriptionGoTo = self.create_subscription(
            Int32,
            'setPosition',
            self.go_to,
            10)
        self.subscriptionSpeed = self.create_subscription(
            Int32,
            'setVelocity',
            self.set_speed,
            10)
        self.subscriptionGoTo
        self.subscriptionSpeed
        self.subscriptionInput  # prevent unused variable warnings
        # edit global logger object with node logger
        self.declare_parameter("Address")
        global logger
        logger = self.get_logger()
        adrNum = self.get_parameter('Address').get_parameter_value()
        address = adrNum.integer_value
        # Create tic object with with motor on /dev/i2c-1
        bus = SMBus(busNum)
        # Select the I2C address of the Tic (the device number).
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

# Checks current position and message data and sends command accordingly
    def checkRun(self, msg):
        # get current position, increase if w, decrease if s
        logger.info("Changing pos")
        curPosition = tic.get_current_position()
        if (msg.data == 'w'):
            tic.exit_safe_start()
            tic.set_target_position(round(curPosition + incrBit))
        elif(msg.data == 's'):
            tic.exit_safe_start()
            tic.set_target_position(round(curPosition - incrBit))

    # Make tic go to current position in mm
    def go_to(self, msg):
        logger.info(str(tic.get_current_status()))
        if(tic.get_current_status() == 10):
            tic.exit_safe_start()
            tic.set_target_position(round(msg.data*incrBit))

    # Sets speed of tic in rot/min
    def set_speed(self, msg):
        logger.info(str(tic.get_current_status()))
        if(tic.get_current_status() == 10):
            tic.exit_safe_start()
            tic.set_target_speed(round(msg.data*velBit))


def main(args=None):

    rclpy.init(args=args)
    subscriber = keySubscriber()
    try:
        rclpy.spin(subscriber)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        # Stopping motor completely
        logger.info("Shutting down motor %d" % address)
        tic.powerdown()
        subscriber.destroy_node()
        rclpy.shutdown()



# Tests

def motorConnectTest(testAddress):
    try:
        bus = SMBus(busNum)
        # Select the I2C address of the Tic (the device number).
        bus.read_byte_data(testAddress, 0)
        return 0
    except Exception:
        return 1

def motorPositionTest(testAddress):
    try:
        bus = SMBus(busNum)
        tic = TicI2C(bus, testAddress)
        #tic.reset_motor()
        retVal = tic.get_current_position()
        return 0
    except Exception:
        return 1

def motorHomeTest(testAddress):
    try:
        bus = SMBus(busNum)
        tic = TicI2C(bus, testAddress)
        retVal = tic.go_home()
        return 0
    except Exception:
        return 1

def doneHoming(testAddress):
    try:
        bus = SMBus(busNum)
        tic = TicI2C(bus, testAddress)
        retVal = tic.get_current_flags()
        while ((retVal >> 0 & 0x10) == 1):
            retVal = tic.get_current_flags()
            time.sleep(0.1)
        # Checking to make sure we are homed
        return not (tic.get_current_position() == 0)
    except Exception:
        return 1

def velocityTest(testAddress):
    try:
        bus = SMBus(busNum)
        tic = TicI2C(bus, testAddress)
        tic.set_target_speed(200)
        time.sleep(0.5)
        retVal = tic.get_current_velocity()
        assert not (retVal == 200)
    except Exception:
        return 1

def velocityStopTest(testAddress):
    try:
        bus = SMBus(busNum)
        tic = TicI2C(bus, testAddress)
        tic.set_target_speed(0)
        time.sleep(0.5)
        retVal = tic.get_current_velocity()
        assert not (retVal == 0)
    except Exception:
        return 1


# Main function
if __name__ == '__main__':
    try:
        main()
    except Exception:
        pass
