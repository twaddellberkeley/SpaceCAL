import time
from threading import Thread
import threading
from functools import partial

from interfaces.action import Level
from interfaces.srv import GuiDisplay
from queue import Queue
from rclpy.node import Node
from rclpy.action import ActionClient

LEVEL = [18, 91, 164, 237, 310]
MAX_HEIGHT = 310
MIN_HEIGHT = 0

STATE_STOPPED = 0x00
STATE_READY = 0x01
STATE_MOVING = 0x10
STATE_PRINTING = 0x100


class LevelController(Node):
    def __init__(self, parent):
        super().__init__('client_level_controller_node')
        self._state = "off"   # [moving, home, error]
        self._curr_position = 0
        self._req_position = Queue()
        self._feedback_position = 0
        self._curr_level = 0
        self._req_level = Queue()
        self._is_moving = False
        # is true if a level is set or requested
        self._was_level_req = Queue()
        self._is_position_uncertain = True

        #***** Initialize Goalhandle to cancel goal if needed ******#
        self.goal_handle = None

        #***** Clients Action *******#
        self._action_client = ActionClient(
            parent, Level, "level_motor_action_srv")

        #***** Clients Service *******#
        self.gui_cli = self.create_client(
            GuiDisplay, "gui_display_srv")

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
        _was_level_req = False
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
        elif split_cmd[2] == "next":
            numberOfLevels = len(LEVEL)
            # loop through the levels
            next_level = (self._curr_level + 1) % numberOfLevels
            self._req_level.put(next_level)
            _was_level_req = True
            position_cmd = LEVEL[next_level]
        elif split_cmd[2] == "stop":
            self.stop_motors()
        else:
            level = int(split_cmd[2])
            assert level >= 0 and level < len(
                LEVEL), "[ERROR]: incorrect level input"
            position_cmd = LEVEL[level]
            self._req_level.put(level)
            _was_level_req = True

        if _was_level_req == True:
            self._was_level_req.put(1)
        else:
            self._was_level_req.put(0)
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
        self._req_position.put(order)
        # Connect to server for _action_client
        self._action_client.wait_for_server()

        # Send goal message with feedback callback function for updates
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        # Add a done Callback function to handle the final answer
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        # Check if goal was accepted
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            # TODO: send messge to gui that it needs to home first
            return
        self.get_logger().info('Goal accepted :)')
        self.goal_handle = goal_handle
        # Update level state to moving
        self._is_moving = True
        gui_req = GuiDisplay.Request()
        gui_req.cmd = "display+state"
        gui_req.display_name = "level-state"
        gui_req.display_msg = "Moving"
        gui_req.state = STATE_MOVING
        self.gui_cli_req(gui_req)

        # get the result later
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

        # TODO: Send message to let the gui know level is moving    ######################
        # self.client_req("display-levelstate-moving-gui", "display", None)

    def get_result_callback(self, future):
        # This function is call when the level action has finished
        result = future.result().result
        gui_req = GuiDisplay.Request()
        # This should display all the time unless we go to a specific level
        gui_req.display_msg = "Home"
        # TODO: Handle error here!!! when the level is not homed

        # Check result
        self._curr_position = result.height
        if self._req_position.get() != self._curr_position:
            self.get_logger().warning("did not get exact requested position")

        
        self.get_logger().info('Current position: {0}'.format(result.height))
        gui_req.display_msg = "Position " + str(self._curr_position) + " mm"
        # TODO: Send message to let the gui know the leve stoped moving at a given position
        if self._was_level_req.get():
            # update current level
            self._curr_level = self._req_level.get()
            if LEVEL[self._curr_level] != self._curr_position:
                self.get_logger().warning(
                    "controller did not make it to the correct height for given level")


            # update display message
            gui_req.display_msg = str(self._curr_level)
            # display_msg = "display-level-" + \
            #     str(self._curr_level) + "-gui"
            # self.client_req(display_msg, "display", None)

        self._is_moving = False

        
        gui_req.cmd = "display+state"
        gui_req.display_name = "level-state"
        gui_req.state = STATE_STOPPED
        self.gui_cli_req(gui_req)

        self.get_logger().info("Also made it here")

        # TODO: Send message to let the gui know level is moving    ######################
        # self.client_req("display-levelstate-stopped-gui", "display", None)

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

    def gui_cli_req(self, req):
        ############# temp variables for debug ###############
        # time_to_wait = 4
        # count = 0
        ######################################################

        while not self.gui_cli.wait_for_service(timeout_sec=.2):
            self.get_logger().info('service not available, waiting again...')
            # if count > time_to_wait:
            #     return
            # count += 1
        self.get_logger().warning(
            "[Sending to Gui]: Sending request cmd: %s" % (req.cmd))

        future = self.gui_cli.call_async(req)
        # Add callback to receive response
        future.add_done_callback(partial(self.gui_display_future_callback))
        ################################################################
        self.get_logger().debug('Waiting on async...')   # DEBUG message

    def gui_display_future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().debug('Print Gui Display Result.msg: %s' % (response.msg))
            if response.err != 0:
                self.get_logger().error('[ERROR]: gui display response error %s' %
                                        (response.msg))
        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished async call....')

    ############################# Utilities ##################################
    def stop_motors(self):
        # if self.goal_handle != None:
        #     future = self.goal_handle.cancel_goal_async()
        print("Need to implement")
        #TODO: maybe send a halt cmd to the motors
