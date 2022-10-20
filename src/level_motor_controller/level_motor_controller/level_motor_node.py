import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from interfaces.action import Level


class LevelActionServer(Node):

    def __init__(self):
        super().__init__('level_action_server')
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