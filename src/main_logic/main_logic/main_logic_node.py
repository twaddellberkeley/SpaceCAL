
import time
from threading import Thread
import threading
from xml.etree.ElementTree import tostring
from functools import partial
from queue import Queue

from interfaces.srv import GuiDisplay, GuiInput, Projector, Video, MotorSrv
from interfaces.action import Level


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

commands = ["proj-on-all", "proj-off-all", "rotate-vile-30"]
LEVEL = [18, 91, 164, 237, 310]
MAX_HEIGHT = 310
MIN_HEIGHT = 0


class Motor():

    def __init__(self, id):
        super().__init__()
        self._id = id
        self._status = "off"
        self._speed = 0
        self._position = 0


class LevelState():
    def __init__(self):
        self._status = "off"   # [moving, home, error]
        self._curr_position = 0
        self._level = 0
        self._req_level = -1
        self._is_moving = False
        self._reqLevel = False
        self._req_position = -1
        self._feedback_position = 0


class Printer():
    def __init__(self, id):
        super().__init__()
        self._id = id
        self._status = "off"
        self._isProjOn = False
        self._isLedOn = False
        self._isVideoOn = False
        self._motor = Motor(id)
        self._pi_videos = []
        self._pi_queue = Queue()
        self._queue_index = 0


class MainLogicNode(Node):

    def __init__(self):
        super().__init__('main_logic_node')

        ################## Services ####################
        #***** Servers ******#
        self.gui_srv = self.create_service(
            GuiInput, 'gui_input_srv', self.gui_input_callback)

        #***** Clients *******#
        self._action_client = ActionClient(
            self, Level, "level_motor_action_srv")
        # self.proj_cli = self.create_client(Projector, 'projector_srv')
        # self.gui_cli = self.create_client(GuiDisplay, 'gui_display_srv')
        # self.video_cli = self.create_client(Video, 'video_srv')

        #***** Initialize printer objects ******#
        self._printer = [Printer(0), Printer(
            1), Printer(2), Printer(3), Printer(4)]

      
        self.pi_controller_logic("pi-get-videos-all")
        self.pi_controller_logic("pi-get-queue-all")

        #***** Initialize Level Controller ******#
        self._levelState = LevelState()

        #***** Initialize Goalhandle to cancel goal if needed ******#
        self.goal_handle = None

    def gui_input_callback(self, request, response):
        self.get_logger().info('Incoming Gui Request\ncmd: %s ' % (request.cmd))
        # Decode gui input command
        cmd_list = self.decode_gui_cmd(request.cmd)
        # dispach the commands
        res = self.dispatch_gui_cmd(cmd_list)

        response.err = res
        response.msg = "request is is being proccess"
        return response

    def decode_gui_cmd(self, raw_cmd):
        self.get_logger().info('Decoding Gui Command:  %s\n' % raw_cmd)
        # Projector and Level Commands
        ctrls = {"level_cmds": [], "motor_cmds": [],
                 "proj_cmds": [], "pi_cmds": []}
        # Split comnands
        cmd_list = raw_cmd.split("_")
        for cmd in cmd_list:
            split_cmd = cmd.split("-")
            if "proj" == split_cmd[0]:
                # extract project command
                # ctrls["proj_cmds"].append(cmd[len("proj-"):len(cmd)])
                ctrls["proj_cmds"].append(cmd)
                ##################################################################################
                self.get_logger().debug('******** first cmd in proj_cmds: %s\n' %
                                        ctrls["proj_cmds"][0])  # DEBUG MESSAGE
                ##################################################################################
            elif "pi" == split_cmd[0]:
                # extract pi command
                ctrls["pi_cmds"].append(cmd)
                ##################################################################################
                self.get_logger().debug('******** first cmd in pi_cmds: %s\n' %
                                        ctrls["pi_cmds"][0])  # DEBUG MESSAGE
                ##################################################################################
            elif "motor" == split_cmd[0]:
                # extract motor command
                ctrls["motor_cmds"].append(cmd)
                ##################################################################################
                self.get_logger().debug("******** first cmd in motor_cmds: %s\n" %
                                        ctrls["motor_cmds"][0])  # DEBUG MESSAGE
                ##################################################################################
            elif "level" == split_cmd[0]:
                # extract level commands
                ctrls["level_cmds"].append(cmd)
                ##################################################################################
                self.get_logger().debug('******** first cmd in level_cmds: %s' %
                                        ctrls["level_cmds"][0])  # DEBUG MESSAGE
                ##################################################################################
        return ctrls

    def dispatch_gui_cmd(self, ctrls):
        """ TODO: Add fucntions description """
        # It reads from a list of commands and dispatches them in their own thread
        ########################################################################
        self.get_logger().debug('Dispatching Gui Commands...\n')  # DEBUG MESSGE
        ########################################################################

        if len(ctrls["level_cmds"]) != 0:
            # Send command to level controller logic
            for cmd in ctrls["level_cmds"]:
                self.level_controller_logic(cmd)
                # Wait for the platform to stop moving
                while self._levelState._is_moving:
                    time.sleep(.5)

        if len(ctrls["motor_cmds"]) != 0:
            # Send commadns to motor controller logic
            for cmd in ctrls["motor_cmds"]:
                self.motor_controller_logic(cmd)

        if len(ctrls["proj_cmds"]) != 0:
            # Send commands to projector controller logic
            for cmd in ctrls["proj_cmds"]:
                self.proj_controller_logic(cmd)

        if len(ctrls["pi_cmds"]) != 0:
            # Send commands to pi controller logic
            for cmd in ctrls["pi_cmds"]:
                self.pi_controller_logic(cmd)

        ###################################################################################
        self.get_logger().debug('Finished Dispatching Gui Commands...\n')    # DEBUG MESSGE
        ###################################################################################
        return 0

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
            position_cmd = self._levelState._curr_position + int(split_cmd[3])
        elif split_cmd[2] == "down":
            assert len(split_cmd) == 4, "ERROR: command format error"
            position_cmd = self._levelState._curr_position - int(split_cmd[3])
        elif split_cmd[2] == "home":
            position_cmd = 0
        else:
            level = int(split_cmd[2])
            assert level >= 0 and level < len(
                LEVEL), "[ERROR]: incorrect level input"
            position_cmd = LEVEL[level]
            self._levelState._req_level = level
            self._levelState._reqLevel = True

        # Check position is within bounded allowed
        if position_cmd < MIN_HEIGHT:
            position_cmd = MIN_HEIGHT
        elif position_cmd > MAX_HEIGHT:
            position_cmd = MAX_HEIGHT

        # Send level command
        self.send_height_goal(position_cmd)

    def motor_controller_logic(self, motor_cmd):
        """ TODO: Description of this function """
        # TODO: add any logic if neccessary when turning motors on or off
        assert (motor_cmd != None)
        client = "motor"
        index = 1
        request = MotorSrv.Request()
        split_cmd = motor_cmd.split("-")
        request.cmd = split_cmd[index]
        if split_cmd[index] == "on":
            request.speed = int(split_cmd[index+1])
        elif split_cmd[index] == "off":
            request.speed = 0
        self.client_req(motor_cmd, client, request)
           

    def proj_controller_logic(self, proj_cmd):
        """This Function takes a command PROJ_CMD and implements the logic necessary to complete the comand
                - **proj-on-<#>**: turns on the projector number # and parks the DMD
                - **proj-off-<#>**: turns off the projector number # and parks the DMD
                - **proj-led-on-<#>**: turns the projector number #’s led on
                - **proj-led-off-<#>**: turns the projector number #’s led off
             """
        assert (proj_cmd != None)
        index = 1
        client = "proj"
        split_cmd = proj_cmd.split("-")
        req = Projector.Request()
        if split_cmd[index] == "on":
            # Turn projector on
            req.cmd = split_cmd[index]
            self.client_req(proj_cmd, client, req)
        elif split_cmd[index] == "off":
            # # Trun projector off
            # if self.is_proj_led_on(split_cmd[index + 1]):
            #     # If the led is on turn it off first
            #     req.cmd = "led-off"
            #     self.client_req("proj-led-off-" + split_cmd[index + 1], client, req)
            req.cmd = split_cmd[index]
            self.client_req(proj_cmd, client, req)
        elif split_cmd[index] == "led":
            req.cmd = split_cmd[index] + "-" + split_cmd[index + 1]
            # if split_cmd[index + 1] == "on":
                # # Turn led on
                # if not self.is_proj_on(split_cmd[index + 2]):
                #     # Verify projector is on else turn it on first
                #     self.client_req("", client, None)
            self.client_req(proj_cmd, client, req)
            # else:
            #     self.get_logger().error('Command not recognized in project logic: ')
        else:
            self.get_logger().error('Command not recognized in project logic: ')

    def pi_controller_logic(self, pi_cmd):
        # - **pi-get-videos-<#>**: gets all the videos available in the pi #
        # - **pi-get-queue-<#>**: get the video queue for pi #
        # - **pi-play-<videoName>-<#>**: play **videoName** from pi #
        # - **pi-stop-video-<#>**: stop playing video from pi #
        # - **pi-play-queue-<#>**: play video queue from pi #
        # - **pi-stop-queue-<#>**: stop video queue from pi # (this will exit the queue and wont remember where it stoped)
        # - **pi-pause-queue-<#>**: pauses the queue from pi # (This will stop the current print and get ready to play the next video.)
        assert (pi_cmd != None)
        # TODO: Implement logic if needed. As of now it sends every message to the video controller
        index = 1
        client = "pi"
        split_cmd = pi_cmd.split("-")
        # TODO: we could posible do a get request of all the videos from the pi at start-up and save it to the
            #  to the projector structure, since this wont change during printing.
        req = Vidoe.Request()
        if split_cmd[index] == "get":
            req.cmd = split_cmd[index] + "-" + split_cmd[index+1]
        elif split_cmd[index] == "play":
            req.cmd = split_cmd[index] 
            queue_index = 0
            # Send Command to all printers
            if split_cmd[-1] == "all":
                for i in range(5):
                    if split_cmd[index+1] == "queue":

                        # get next queue item from queue
                        queue_index = self._printer[i]._queue_index
                        # Verify that there is an item in the queue
                        self.is_valid_queue_index(queue_index)
                        # Get file name from queue
                        req.file_name = self._printer[i]._pi_queue[queue_index]
                        # convert to singel play request
                        cmd = "pi-play-" + req.file_name + "-" + int(i) 
                        # Send request
                        self.client_req(cmd, client, req)
                        # update queue index to the next video
                        self._printer[i]._queue_index += 1
                    else:
                        # Get index from command
                        queue_index = int(split_cmd[index+1])
                        # Verify there is an item in the queue
                        self.is_valid_queue_index(queue_index)
                        # Get file name from queue
                        req.file_name = self._printer[i]._pi_queue[queue_index]
                        # Get construct new command 
                        cmd = "pi-play-" + req.file_name + "-" + int(i) 
                        # Send command
                        self.client_req(cmd, client, req)
                    
            # Send command to a single printer
            elif split_cmd[index+1] == "queue":
                # Get printer number
                printer_num = int(split_cmd[index+2])
                # Get next queue index from printer
                queue_index = self._printer[printer_num]._queue_index
                # Get video from printer
                req.file_name = self._printer[printer_num]._pi_queue[queue_index]
                self.client_req(cmd, client, req)
                self._printer[printer_num]._queue_index += 1
                
            else:
                req.file_name = split_cmd[index+1]
                self.client_req(cmd, client, req)
                
        elif split_cmd[index] == "stop":
            req.cmd = split_cmd[index]
            self.client_req(pi_cmd, client, req)
        # TODO: we need to really define what pausing a video really means
        elif split_cmd[index] == "pause":
            req.cmd = "stop"
            self.client_req(pi_cmd, client, req)
        

    ################################################ Client Request ##############################################
    def client_req(self, cmd, client, req):
        # times to try to connect to server
        times_to_connect = 0

        ### Select client to send request ###
        if client == "proj":
            srv = Projector
            topic = "projector_srv_"
            callback_func = self.proj_future_callback
        elif client == "pi":
            srv = Video
            topic = "pi_video_srv_"
            callback_func = self.pi_future_callback
        elif client == "motor":
            srv = MotorSrv
            topic = "print_motor_srv_"
            callback_func = self.motor_print_future_callback
        elif client == "display":
            srv = GuiDisplay
            topic = "gui_display_srv"
            callback_func = self.gui_display_future_callback

        # TODO: wher do we recod metadata?

        #### Send request to all five printers ####
        if cmd.split("-")[-1] == "all":
            self.cli = [None] * 5
            for i in range(5):
                #***** Clients *******#
                self.cli[i] = self.create_client(srv, topic + str(i))
                while not self.cli[i].wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')

                # TODO: May need self.future instead of future
                # if client == "pi" and req.cmd == "play":
                #     queue_index = int(req.file_name)
                #     req.file_name = self._printer[i]._pi_queue[queue_index]
                future = self.cli[i].call_async(req)
                future.add_done_callback(partial(callback_func))
                ################################################################
                self.get_logger().debug('Waiting on async...')  # DEBUG message
                ################################################################

        #### Send request to one printer ####
        elif cmd.split("-")[-1] in ['0', '1', '2', '3', '4']:
            #***** Clients *******#
            i = int(cmd.split("-")[-1])
            self.cli = self.create_client(srv, topic + cmd.split("-")[-1])
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            # Populate request
            future = self.cli.call_async(req)
            future.add_done_callback(partial(callback_func))
            ################################################################
            self.get_logger().debug('Waiting on async...')   # DEBUG message
            ################################################################

        #### Send request to gui ####
        elif cmd.split("-")[-1] == "gui":
            count = 0
            #***** Clients *******#
            split_cmd = cmd.split("-")
            self.cli = self.create_client(srv, topic)
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
                if count > times_to_connect:
                    self.get_logger().warning('Could not connect to Gui Service')
                    break
                count += 1
            # Populate request TODO: this needs to change when we add gui controller
            request = srv.Request()
            request.cmd = cmd
            request.display_name = split_cmd[1]
            request.display_msg = split_cmd[2]
            future = self.cli.call_async(request)
            future.add_done_callback(partial(callback_func))
            ################################################################
            self.get_logger().debug('Waiting on async...')   # DEBUG message
            ################################################################

        # ERROR: does not recognize the commmad
        else:
            self.get_logger().error('Command not recognized: %s\n' % (request.cmd))

    ################################################ Action Client ##############################################
    # This function sends the desired height to the level controller
    def send_height_goal(self, order):
        assert type(order) == type(3), "[ERROR]: wrong type"
        # Create Level message
        goal_msg = Level.Goal()
        # Update messge
        goal_msg.order = order
        self._levelState._req_position = order
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
        self._levelState._is_moving = True

        # get the result later
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

        # TODO: Send message to let the gui know level is moving    ######################
        self.client_req("display-levelstatus-moving-gui", "display", None)

    def get_result_callback(self, future):
        # This function is call when the level action has finished
        result = future.result().result

        # TODO: Handle error here!!! when the level is not homed

        # Check result
        self._levelState._curr_position = result.height
        if self._levelState._req_position != result.height:
            self.get_logger().warning("did not get exact requested position")

        self._levelState._req_position = -1
        self.get_logger().info('Current position: {0}'.format(result.height))

        # TODO: Send message to let the gui know the leve stoped moving at a given position
        if self._levelState._reqLevel:
            if LEVEL[self._levelState._req_level] != result.height:
                self.get_logger().warning(
                    "controller did not make it to the correct height for given level")
            # reset the request leve flag
            self._levelState._reqLevel = False
            # update current level
            self._levelState._level = self._levelState._req_level
            # reset requested level
            self._levelState._req_level = -1
            # update display message
            display_msg = "display-level-" + \
                str(self._levelState._level) + "-gui"
            self.client_req(display_msg, "display", None)

        self._levelState._is_moving = False
        self.client_req("display-levelstatus-stopped-gui", "display", None)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._levelState._curr_position = feedback.feedback_height

        if _levelState._curr_position > 100:
            cancel_goal_future = self._action_client.cancel_goal_async()
            cancel_goal_future.add_done_callback()
        # TODO: posible publish height

        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.feedback_height))

    ############################################ End of Acton Client ##################################################

    # TODO: make sure to remember to update the project and motor classes from the response data

    def proj_future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Projector status %s' % (response.status))
            if response.err == 0:
                self._printer[0]._isLedOn = response.is_led_on
                self._printer[0]._isVideoOn = response.is_video_on
                self.get_logger().info('response.is_led_on %s' %
                                       ("True" if response.is_led_on else "False"))
        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished async call....')
        self.get_logger().info('Printer_1._isLedOn %s' %
                               ("True" if self._printer[0]._isLedOn else "False"))

    def pi_future_callback(self, future):
        try:
            # TODO: update --> self._printer[i]._queue_index
            res = future.result()
            self.get_logger().info('Pi Video status %s' % (res.status))
            # int32 err
            # string[] videos
            # string[] queue
            # string msg
            # string status 
            # bool is_video_on 

            if len(res.videos) > 0:
                self._printer[res.id]._pi_videos = res.videos
            if len(res.queue) > 0:
                self._printer[res.id]._pi_queue = res.queue
                
            self.get_logger().info('res.is_led_on %s' %
                                       (res.msg))
        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished async call....')
        

    def motor_print_future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Print Motor status %s' % (response.status))
            if response.err == 0:
                self._printer[0]._motor._speed = response.set_speed
                self.get_logger().info('response.set_speed %s' %
                                       (response.set_speed))
        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished async call....')
        self.get_logger().info('Printer_1._isLedOn %s' %
                               ("True" if self._printer[0]._isLedOn else "False"))

    def gui_display_future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().debug('Print Gui Display Result.msg: %s' % (response.msg))
            if response.err != 0:
                self.get_logger().error('[ERROR]: gui display response error %s' %
                                        (response.msg))
            self.get_logger().info('Gui Process Command Succesfully: %s' % (response.msg))
        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished async call....')

    #********* Utility functions ***********#

    def is_proj_on(self, proj_num):
        assert type(proj_num) == type("")
        if proj_num == "all":
            for p in range(5):
                if not self._printer[p]._isProjOn:
                    return False
            return True
        else:
            return self._printer[int(proj_num)]._isProjOn

    def is_proj_led_on(self, proj_num):
        assert type(proj_num) == type("")
        if proj_num == "all":
            for p in range(5):
                if not self._printer[p]._isLedOn:
                    return False
            return True
        else:
            return self._printer[int(proj_num)]._isLedOn

    def is_valid_queue_index(self, index):
        queue_index = int(split_cmd[index+1])
        for i in rage(5):
            assert queue_index < len(self._printer[i]._pi_queue), "Queue index out of range"


def main(args=None):
    rclpy.init(args=args)
    #print('In Main thread with thread_id: %d\n' % threading.get_native_id())
    service = MainLogicNode()
    # service.send_height_goal(10)

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
