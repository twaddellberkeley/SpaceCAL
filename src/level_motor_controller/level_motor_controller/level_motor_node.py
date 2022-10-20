import time
import threading
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from smbus2 import SMBus

from interfaces.action import Level
from .submodules.motor_controller import Motor


class LevelActionServer(Node):
    

    def __init__(self):
        super().__init__('level_action_server')
        index = 0
        num_motors = 4
        start_address = 14
        self.motor_num = [None] * num_motors
        self.address = [None] * num_motors
        self.motors = [None] * num_motors
        self.stayAlive = [None] * num_motors

        ###############################################################################################################
        while index < num_motors:                                       #######                                ########
            m = "motor_number_" + str(index)                            #######                                ########
            a = "address_" + str(index)                                 ######  ROS Parameters: "motor_number"  #######
            self.declare_parameter(m, index)                            #######                 "address"      ########
            self.declare_parameter(a, start_address)                    #######                                ########
            self.motor_num[index] = self.get_parameter(m).value         #######                                ########
            self.address[index] = self.get_parameter(a).value           #######                                ########
            index += 1                                                  #######                                ########
            start_address += 1                                          #######                                ########
        ###############################################################################################################

        ###############################################################################################################
        # self.declare_parameter("motor_number_0", 0)                        #######                                #######
        # self.declare_parameter("motor_number_1", 1)                        #######                                #######
        # self.declare_parameter("motor_number_2", 2)                        #######                                #######
        # self.declare_parameter("motor_number_3", 3)                        #######                                #######
        # self.declare_parameter("address_0", 14)                            ######  ROS Parameters: "motor_number"  ######
        # self.declare_parameter("address_1", 15)                            ######  ROS Parameters: "motor_number"  ######
        # self.declare_parameter("address_2", 16)                            ######  ROS Parameters: "motor_number"  ######
        # self.declare_parameter("address_3", 17)                            ######  ROS Parameters: "motor_number"  ######
        # self.motor_num_0 = self.get_parameter('motor_number_0').value        ######                   "address"      ######
        # self.motor_num_1 = self.get_parameter('motor_number_1').value        ######                   "address"      ######
        # self.motor_num_2 = self.get_parameter('motor_number_2').value        ######                   "address"      ######
        # self.motor_num_3 = self.get_parameter('motor_number_3').value        ######                   "address"      ######
        # self.address_0 = self.get_parameter("address_0").value               #######                                #######
        # self.address_1 = self.get_parameter("address_1").value               #######                                #######
        # self.address_2 = self.get_parameter("address_2").value               #######                                #######
        # self.address_3 = self.get_parameter("address_3").value               #######                                #######
        ###############################################################################################################

        # Create a bus object with with motor on /dev/i2c-1
        bus = SMBus(1) # TODO: figureout how to get the bus number automatically

        index = 0
        while index < num_motors:
            self.motors[index] = Motor(self.motor_num[index], bus, self.address[index], self.get_logger())
            self.motors[index].reset_motor()
            index += 1
            time.sleep(0.1)

        # Create a thread to keep the motors energyzed in ordered to mantain its position
        index = 0
        while index < num_motors:
            self.stayAlive[index] = threading.Thread(target=self.motors[index].stay_alive)          
            # Allow the thread to run in the backgorund
            self.stayAlive[index].daemon = True
            self.stayAlive[index].start()
            self.motors[index].exit_safe_start()
            self.motors[index].energize()

        # Create an action server
        self._action_server = ActionServer(
            self,
            Level,
            'level_motor_action_srv',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Level.Feedback()
        feedback_msg.feedback_height = self.getCurrHeight()     # TODO get current height from motors or motor class

        for i in range(1, goal_handle.request.order):
            feedback_msg.feedback_height += 1
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.feedback_height))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Level.Result()
        result.height = feedback_msg.feedback_height
        return result

    def getCurrHeight(self):
        return 0

def main(args=None):
    print("Hello From LevelActioServer")
    rclpy.init(args=args)

    level_action_server = LevelActionServer()

    rclpy.spin(level_action_server)


if __name__ == '__main__':
    main()