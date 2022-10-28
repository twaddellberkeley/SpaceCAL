import time
import threading
from turtle import position
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from smbus2 import SMBus

from interfaces.action import Level
from .submodules.motor_controller import Motor


class LevelActionServer(Node):
    def __init__(self):
        super().__init__('level_action_server')

        num_motors = 4
        start_address = 14
        self.motor_num = [None] * num_motors
        self.address = [None] * num_motors
        self.motors = [None] * num_motors
        self.stayAlive = [None] * num_motors

        ############################################# ROS Parameters #################################################
        index = 0
        while index < num_motors:
            m = "motor_number_" + str(index)
            a = "address_" + str(index)
            self.declare_parameter(m, index)
            self.declare_parameter(a, start_address)
            self.motor_num[index] = self.get_parameter(m).value
            self.address[index] = self.get_parameter(a).value
            index += 1
            start_address += 1
        ###############################################################################################################

        # Create a bus object with with motor on /dev/i2c-1
        bus = SMBus(1)

        # Initialize the motors
        index = 0
        while index < num_motors:
            self.motors[index] = Motor(
                self.motor_num[index], bus, self.address[index], self.get_logger())
            self.motors[index].reset_motor()
            index += 1
            time.sleep(0.1)

        # Create a thread to keep the motors energyzed in ordered to mantain its position
        index = 0
        while index < num_motors:
            self.stayAlive[index] = threading.Thread(
                target=self.motors[index].stay_alive)
            # Allow the thread to run in the backgorund
            self.stayAlive[index].daemon = True
            self.stayAlive[index].start()
            self.motors[index].exit_safe_start()
            self.motors[index].energize()
            # TODO: Uncoment when spped is confirmed
            # self.motors[index].setMaxSpeed()
            index += 1

        # Create an action server
        self._action_server = ActionServer(
            self,
            Level,
            'level_motor_action_srv',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Level.Feedback()
        # TODO get current height from motors or motor class
        feedback_msg.feedback_height = self.getCurrHeight()

        self.setPosition(goal_handle.request.order)

        while self.getCurrHeight() < goal_handle.request.order:
            feedback_msg.feedback_height = self.getCurrHeight()
            self.get_logger().info('Feedback: {0}'.format(
                feedback_msg.feedback_height))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(.1)

        goal_handle.succeed()
        result = Level.Result()
        result.height = self.getCurrHeight()
        self.get_logger().info("Motor Final positon %d" % (result.height))
        return result

    def setPosition(self, position):
        for i in range(4):
            if position == 0:
                self.motors[i].goHome()
            else:
                self.motors[i].setTargetPosition(position)

    def getCurrHeight(self):
        positions = [None]*4
        for i in range(4):
            positions[i] = self.motors[i].getCurrentPosition()
            self.get_logger().info("Current Motor Height-- num: %d  height: %d" %
                                   (i, positions[i]))
        # Convert position to actual height in mm
        height = int(sum(positions)/4)
        return height


def main(args=None):
    print("Hello From LevelActioServer")
    rclpy.init(args=args)

    level_action_server = LevelActionServer()

    rclpy.spin(level_action_server)


if __name__ == '__main__':
    main()
