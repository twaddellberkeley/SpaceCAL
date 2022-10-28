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


# Author: Taylor Waddell and Christian Castaneda
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
from smbus2 import i2c_msg

# Tic class with generic motor controls

# Step veloicty to achieve 1rot/min
VEL_BIT = 1024000000/60  # Max 500000000

# Increments: Tic 36v4: current limit, in units of 71.615 mA
# Range: Tic 36v4: 0 mA to 9095 mA
CURRENT_INCREMENT = 71.615

# Pulse to move moto
# 1 mm per unit sent, max 310
INCR_BIT = 1280  # Is 1mm per bit unit

# Calculation 310/8sec => 39mm/s => 1280 * 39 => 49920
MAX_SPEED = 49920  # this is a speed for 39mm per sec


class TicI2C(object):
    # Init function, takes bus and address
    def __init__(self, bus, address, logger):
        # Bus object and address object passed down
        self.bus = bus
        self.address = address
        self.logger = logger
        # See if we can read a bit to test if motor is open
# to commet for testing
        try:
            self.bus.read_byte_data(self.address, 0)
        except Exception:
            self.logger.error(
                "Could not open motor controller on address %d" % self.address)
            exit(0)
###########################################################################################
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
        self.logger.info("Target %d" % target)
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

    # Sets maximum speed until next reset
    def set_max_speed(self, speed):
        command = [0xE6,
                   speed >> 0 & 0xFF,
                   speed >> 8 & 0xFF,
                   speed >> 16 & 0xFF,
                   speed >> 24 & 0xFF]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # The Tic 36v4 represents current limits using numbers between 0 and 127
    # Set current limit
    def set_current_limit(self, value):
        command = [0x40, value]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Tells motor to home
    def go_home(self):
        command = [0x97, 0]
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
        self.logger.warn("Starting stay alive")
        while rclpy.ok():
            try:
                self.clear_Timeout()
            except Exception:
                self.logger.warn("MOTOR DISCONNECTED %d" % self.address)
                self.logger.warn("SHUTTING DOWN NODE")
                rclpy.shutdown()
                exit(0)
            time.sleep(.5)
        self.logger.warn("Ended stay alive")

    # Gets one or more variables from the Tic.
    def get_variables(self, offset, length):
        write = i2c_msg.write(self.address, [0xA1, offset])
        read = i2c_msg.read(self.address, length)
        self.bus.i2c_rdwr(write, read)
        return list(read)

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
        if velocity >= (1 << 31):
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

    def get_current_step(self):
        b = self.get_variables(0x49, 1)
        status = b[0]
        return status

    def get_current_plannig_mode(self):
        b = self.get_variables(0x09, 1)
        status = b[0]
        return status


class Motor(TicI2C):

    def __init__(self, id, bus, address, logger):
        super().__init__(bus, address, logger)
        self._id = id
        self._status = "off"    # posible status: [off, on, running, energized]
        self._speed = 0
        self._logger = logger
        self._curr_position = 0

    # Resets Motor
    def resetMotor(self):
        self.reset_motor()

    # Homes the motors
    def goHome(self):
        if (self.get_current_status() == 10):
            self.exit_safe_start()
            self.go_home()

    # Halts the motors
    def haltMotor(self):
        self.halt_motor()

    # Sets target speed
    def setTargetPosition(self, target):
        # Convert to pulses per sec
        position = int(target * INCR_BIT)
        if (self.get_current_status() == 10):
            self.exit_safe_start()
            self.set_target_position(position)

    # Sets speed of tic in rot/min
    def setTargetSpeed(self, speed):
        if (self.get_current_status() == 10):
            self.exit_safe_start()
            self.set_target_speed(round(speed*VEL_BIT))

    # Sets Max Speed
    def setMaxSpeed(self):
        self.set_max_speed(MAX_SPEED)

    # The Tic 36v4 represents current limits using numbers between 0 and 127
    # that are linearly proportional to the current limit
    # Increments: Tic 36v4: current limit, in units of 71.615 mA
    # Range: Tic 36v4: 0 mA to 9095 mA
    def setCurrentLimit(self, limit):
        value = 0
        while limit > CURRENT_INCREMENT * value:
            value += 1
        self.set_current_limit(value)

    # Get current velocity rot/min
    def getCurrentVelocity(self):
        velocity = self.get_current_velocity()
        return int(velocity/VEL_BIT)

    # Get current position in mm
    def getCurrentPosition(self):
        return int(self.get_current_position()/INCR_BIT)

    # Get current flags
    def getCurrentFlags(self):
        # • Bit 0: Energized – The Tic’s motor outputs are enabled and if a stepper motor is properly connected, its coils are energized (i.e. electrical current is flowing).
        # • Bit 1: Position uncertain – The Tic has not received external confirmation that the value of its “current position” variable is correct (see Section 5.4).
        # • Bit 2: Forward limit active – One of the forward limit switches is active.
        # • Bit 3: Reverse limit active – One of the reverse limit switches is active.
        # • Bit 4: Homing active – The Tic’s homing procedure is running.
        # • Bits 5–7: reserved
        return self.get_current_flags()

    # Get current status
    def getCurrentStatus(self):
        # • 0: Reset
        # • 2: De-energized
        # • 4: Soft error
        # • 6: Waiting for ERR line
        # • 8: Starting up
        # • 10: Normal
        return self.get_current_status()

    # Get current flags

    # # @Overwrite Tic function for testing
    # def reset_motor(self):
    #     self._logger.info("Reseting Mottor...")
    #     time.sleep(0.1)

    # # @Overwrite Tic function for testing
    # def stay_alive(self):
    #     while rclpy.ok():
    #         try:
    #             time.sleep(10)
    #         except Exception:
    #             self.logger.warn("MOTOR DISCONNECTED %d" % self.address)
    #             self.logger.warn("SHUTTING DOWN NODE")
    #             # rclpy.shutdown()
    #             exit(0)
    #         self._logger.info("Stay alive id %d" % (self._id))
    #         time.sleep(.5)

    # # @Overwrite Tic function for testing
    # def exit_safe_start(self):
    #     self._logger.info("Exiting safe start...")
    #     time.sleep(0.1)

    # # @Overwrite Tic function for testing
    # def energize(self):
    #     self._logger.info("Energizing...")
    #     time.sleep(0.1)
