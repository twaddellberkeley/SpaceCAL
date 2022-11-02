import time
from threading import Thread
import threading
from functools import partial

from interfaces.action import Level

from rclpy.node import Node
from rclpy.action import ActionClient

LEVEL = [18, 91, 164, 237, 310]
MAX_HEIGHT = 310
MIN_HEIGHT = 0


class LevelController(Node):
    def __init__(self):
        super().__init__('level_controller_node')
        self._status = "off"   # [moving, home, error]
        self._curr_position = 0
        self._req_position = -1
        self._feedback_position = 0
        self._curr_level = 0
        self._req_level = -1
        self._is_moving = False
        # is true if a level is set or requested
        self._was_level_req = False
        self._is_position_uncertain = True

        #***** Clients *******#
        self._action_client = ActionClient(
            self, Level, "level_motor_action_srv")

        #***** Initialize Goalhandle to cancel goal if needed ******#
        self.goal_handle = None

    def send_level_cmd(self, cmd):
        self.level_controller_logic(cmd)

    def level_controller_logic(self, level_cmd):
        # The following are the level commands
        """ LEVEL_CMD:
                        level-motors-<level>
                        level-motors-height-<heigh[mm]>
                        level-motors-up-<distance[mm]>
                        level-motors-down-<distance[mm]>
                        level-motors-home               # right now home is position 0
        """
        assert type(level_cmd) == type(""), "Must be string type"

        # The level controller take height in mm so we need to decode the message here from level to mm
        # and or add the current heigh of the levelState
        position_cmd = 0
        level = 0
        split_cmd = level_cmd.split("-")
        if split_cmd[2] == "height":
            assert len(split_cmd) == 4, "ERROR: command format error"
            position_cmd = int(split_cmd[3])
        elif split_cmd[2] == "up":
            assert len(split_cmd) == 4, "ERROR: command format error"
            position_cmd = self._curr_position + int(split_cmd[3])
        elif split_cmd[2] == "down":
            assert len(split_cmd) == 4, "ERROR: command format error"
            position_cmd = self._curr_position - int(split_cmd[3])
        elif split_cmd[2] == "home":
            position_cmd = 0
        else:
            level = int(split_cmd[2])
            assert level >= 0 and level < len(
                LEVEL), "[ERROR]: incorrect level input"
            position_cmd = LEVEL[level]
            self._req_level = level
            self._was_level_req = True

        # Check position is within bounded allowed
        if position_cmd < MIN_HEIGHT:
            position_cmd = MIN_HEIGHT
        elif position_cmd > MAX_HEIGHT:
            position_cmd = MAX_HEIGHT

        # Send level command
        self.send_height_goal(position_cmd)

    ################################################ Action Client ##############################################
    # This function sends the desired height to the level controller

    def send_height_goal(self, order):
        assert type(order) == type(3), "[ERROR]: wrong type"
        # Create Level message
        goal_msg = Level.Goal()
        # Update messge
        goal_msg.order = order
        self._req_position = order
        # Connect to server for _action_client
        self._action_client.wait_for_server()

        # Send goal message with feedback callback function for updates
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        # Add a done Callback function to handle the final answer
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        # Check if goal was accepted
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            # TODO: send messge to gui that it needs to home first
            return
        self.get_logger().info('Goal accepted :)')
        # Update level state to moving
        self._is_moving = True

        # get the result later
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

        # TODO: Send message to let the gui know level is moving    ######################
        # self.client_req("display-levelstatus-moving-gui", "display", None)

    def get_result_callback(self, future):
        # This function is call when the level action has finished
        result = future.result().result

        # TODO: Handle error here!!! when the level is not homed

        # Check result
        self._curr_position = result.height
        if self._req_position != result.height:
            self.get_logger().warning("did not get exact requested position")

        self._req_position = -1
        self.get_logger().info('Current position: {0}'.format(result.height))

        # TODO: Send message to let the gui know the leve stoped moving at a given position
        if self._was_level_req:
            if LEVEL[self._req_level] != result.height:
                self.get_logger().warning(
                    "controller did not make it to the correct height for given level")
            # reset the request leve flag
            self._was_level_req = False
            # update current level
            self._curr_level = self._req_level
            # reset requested level
            self._req_level = -1
            # update display message
            display_msg = "display-level-" + \
                str(self._level) + "-gui"
            self.client_req(display_msg, "display", None)

        self._is_moving = False
        # TODO: Send message to let the gui know level is moving    ######################
        # self.client_req("display-levelstatus-stopped-gui", "display", None)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._curr_position = feedback.feedback_height

        # if self._curr_position > 100:
        #     cancel_goal_future = self._goal_handle.cancel_goal_async()
        #     cancel_goal_future.add_done_callback()
        # TODO: posible publish height

        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.feedback_height))

    ############################################ End of Acton Client ##################################################
