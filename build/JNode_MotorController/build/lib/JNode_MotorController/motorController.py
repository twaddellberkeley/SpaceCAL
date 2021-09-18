#Author: Taylor Waddell
#Link to tic: pololu.com/docs/0J71/12.9
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
from rclpy.node import Node
from std_msgs.msg import String 
from smbus2 import SMBus, i2c_msg
#globl vars
#Tic object
tic = None
#Pulse to move moto
incrBit = 20000
#Logger of the node
logger = None



#Tic class with generic motor controls
class TicI2C(object):
  #Init function, takes bus and address
  def __init__(self, bus, address):
    #Bus object and address object passed down
    self.bus = bus
    self.address = address
    #See if we can read a bit to test if motor is open
    try:
      self.bus.read_byte_data(self.address,0)
    except:
      logger.error("Could not open motor controller on address %d" % self.address)
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
  def set_target_position(self, target):
    command = [0xE0,
      target >> 0 & 0xFF,
      target >> 8 & 0xFF,
      target >> 16 & 0xFF,
      target >> 24 & 0xFF]
    write = i2c_msg.write(self.address, command)
    self.bus.i2c_rdwr(write)
 
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

#Runs the tic motor with a given position
def Run(intIn): 
    tic.exit_safe_start()
    tic.set_target_position(intIn) 
    
class keySubscriber(Node):
  def __init__(self):
    #Create the subscriber node that listens to key input
    super().__init__('KeySubscriber')
    self.subscription = self.create_subscription(
        String,
        'keyinput',
        self.checkRun,
        10)
    self.subscription  # prevent unused variable warnings
    #edit global logger object with node logger
    global logger
    logger = self.get_logger()
    #Create tic object with with motor on /dev/i2c-1
    bus = SMBus(1)    
    # Select the I2C address of the Tic (the device number).
    address = 14
    global tic 
    tic = TicI2C(bus, address)

#Checks current position and message data and sends command accordingly
  def checkRun(self, msg):
    #get current position, increase if w, decrease if s
    curPosition = tic.get_current_position()
    if (msg.data == 'w'):
        Run(curPosition + incrBit)
    elif(msg.data == 's'):
        Run(curPosition - incrBit)

def main(args=None):
    rclpy.init(args=args)
    subscriber = keySubscriber()
    rclpy.spin(subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()

#Main function 
if __name__ == '__main__':
    try:
        main()
    except:
      pass