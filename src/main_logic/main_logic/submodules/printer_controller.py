import time
from datetime import datetime
from threading import Thread
import threading
from functools import partial
from queue import Queue

from interfaces.srv import GuiDisplay, GuiInput, Projector, Video, MotorSrv


import rclpy
from rclpy.node import Node


class Motor():
    def __init__(self, id):
        self._id = id
        self._status = "off"
        self._speed = 0


class PrinterController(Node):
    def __init__(self, id):
        super().__init__("printer_controller_node_" + str(id))
        self._id = id
        self._status = "off"
        self._isProjOn = False
        self._isLedOn = False
        self._isVideoOn = False
        self._isPlayingQueueEmpty = True
        self._motor = Motor(id)
        self._pi_videos = []
        self._pi_queue = []
        self._playing_queue = Queue()

        req = Video.Request()
        req.cmd = "get-videos"
        self.client_req("pi", req)
        req.cmd = "get-queue"
        self.client_req("pi", req)

    def send_motor_cmd(self, cmd):
        self.motor_controller_logic(cmd)

    def send_proj_cmd(self, cmd):
        self.proj_controller_logic(cmd)

    def send_pi_cmd(self, cmd):
        self.pi_controller_logic(cmd)

    ################################################################

    def motor_controller_logic(self, motor_cmd):
        """ TODO: Description of this function 
            - **motor-on-<speed>-<#>**: turns motor **#** with **speed**
            - **motor-off-<#>:** turn motor # off
        """
        # TODO: add any logic if neccessary when turning motors on or off
        assert (motor_cmd != None)
        client = "motor"
        index = 1
        split_cmd = motor_cmd.split("-")
        req = MotorSrv.Request()
        req.cmd = split_cmd[index]
        req.id = self._id

        # Turn motor on
        if split_cmd[index] == "on":
            req.speed = int(split_cmd[index+1])

        # Turn motor off
        elif split_cmd[index] == "off":
            req.speed = 0

        # Command not recognized
        else:
            self.get_logger().error(
                '[motor_controller_logic]: Command not recognized ')
        self.client_req(client, req)

    def proj_controller_logic(self, proj_cmd):
        """This Function takes a command PROJ_CMD and implements the logic necessary to complete the comand
                - **proj-on-<#>**: turns on the projector number # and parks the DMD
                - **proj-off-<#>**: turns off the projector number # and parks the DMD
                - **proj-led-on-<#>**: turns the projector number #’s led on
                - **proj-led-off-<#>**: turns the projector number #’s led off
        """
        assert (proj_cmd != None)
        client = "proj"
        index = 1
        split_cmd = proj_cmd.split("-")
        req = Projector.Request()
        req.id = self._id

        # Turn on HDMI power
        if split_cmd[index] == "on":
            # Turn projector on
            req.cmd = split_cmd[index]

        # Trurn HDMI power off
        elif split_cmd[index] == "off":
            # # Trun projector off
            if self._isLedOn:
                # If the led is on turn it off first
                req.cmd = "led-off"
                self.client_req(client, req)
                # wait to turn led off
                time.sleep(0.5)
            req.cmd = split_cmd[index]

        # Handle LED
        elif split_cmd[index] == "led":
            # Turn LED on
            if split_cmd[index + 1] == "on":
                # Turn led on
                if not self._isProjOn:
                    # Verify projector is on else turn it on first
                    req.cmd = "on"
                    self.client_req(client, req)
                    # wait for projector to be turn on
                    time.sleep(2)
            req.cmd = split_cmd[index] + "-" + split_cmd[index + 1]

        # Command is not recognized
        else:
            self.get_logger().error(
                '[proj_controller_logic]: Command not recognized ')
        self.client_req(proj_cmd, client, req)

    def pi_controller_logic(self, pi_cmd):
        """
            # - **pi-get-videos-<#>**: gets all the videos available in the pi #
            # - **pi-get-queue-<#>**: get the video queue for pi #
            # - **pi-play-<videoName>-<#>**: play **videoName** from pi #
            # - **pi-play-queue-<#>**: play video queue from pi #
            # - **pi-stop-video-<#>**: stop playing video from pi #
            # - **pi-stop-queue-<#>**: stop video queue from pi # (this will exit the queue and wont remember where it stoped)
            # - **pi-pause-queue-<#>**: pauses the queue from pi # (This will stop the current print and get ready to play the next video.)
        """
        assert (pi_cmd != None)
        # TODO: Implement logic if needed. As of now it sends every message to the video controller
        # TODO: we could posible do a get request of all the videos from the pi at start-up and save it to the
        #  to the projector structure, since this wont change during printing.
        client = "pi"
        index = 1
        split_cmd = pi_cmd.split("-")
        req = Video.Request()
        req.id = self._id

        # Process the get command
        if split_cmd[index] == "get":
            req.cmd = split_cmd[index] + "-" + split_cmd[index+1]
            # TODO: return vidoe or queue to gui
            if "queue" in split_cmd:
                # TODO: return queue to gui
                print(self._pi_queue)
                return
            elif "videos" in split_cmd:
                # TODO: return videos to gui
                print(self._pi_videos)
                return
            else:
                self.get_logger().error(
                    "[pi_control_logic]: Invalid Command")

        # Process the play command
        elif split_cmd[index] == "play":
            req.cmd = split_cmd[index]

            # We get a name form the queue
            if split_cmd[index+1] == "queue":
                # Make sure playing queue is not empty
                if self._playing_queue.empty():
                    self._isPlayingQueueEmpty = True
                    return
                req.file_name = self._playing_queue.get()

            # Check if the name is an integer
            elif split_cmd[index+1].isnumeric():
                if not (len(self._pi_queue) > int(split_cmd[index+1])):
                    self.get_logger().error(
                        "[pi_control_logic]: index is out of bound")
                req.file_name = self._pi_queue[int(split_cmd[index+1])]

            # We have a video file name
            elif split_cmd[index+1] in self._pi_videos:
                req.file_name = split_cmd[index+1]

            # We have an invalid file
            else:
                self.get_logger().error(
                    "[pi_control_logic]: Invalid file Name")

        # Process the stop command
        elif split_cmd[index] == "stop":
            req.cmd = split_cmd[index]

        # Reset the queue
        elif split_cmd[index] == "reset":
            self.reset_playing_queue()
            # TODO: send confirmation to gui
            return

        # Command is not recognized
        else:
            self.get_logger().error(
                '[proj_controller_logic]: Command not recognized ')

        self.client_req(client, req)

    ################################################################

    def client_req(self, client, req):

        ### Select client to send request ###
        if client == "proj":
            srv = Projector
            topic = "projector_srv_" + str(self._id)
            callback_func = self.proj_future_callback
        elif client == "pi":
            srv = Video
            topic = "pi_video_srv_" + str(self._id)
            callback_func = self.pi_future_callback
        elif client == "motor":
            srv = MotorSrv
            topic = "print_motor_srv_" + str(self._id)
            callback_func = self.motor_print_future_callback
        elif client == "display":
            srv = GuiDisplay
            topic = "gui_display_srv"
            callback_func = self.gui_display_future_callback

        # TODO: where do we record metadata?
        #***** Clients *******#
        self.cli = self.create_client(srv, topic)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # Record metadate
        req.meta.name = topic
        req.meta.stamp.nanosec = time.time_ns()
        self.get_logger().info("request time stamp in nanosec: %d" %
                               (req.meta.stamp.nanosec))
        # Send async call to server
        future = self.cli.call_async(req)
        # Add callback to receive response
        future.add_done_callback(partial(callback_func))
        ################################################################
        self.get_logger().debug('Waiting on async...')   # DEBUG message

    ################################################################

    def proj_future_callback(self, future):
        try:
            # Metadata meta
            # int32 id
            # string cmd
            # int32 err
            # string msg
            # string status
            # bool is_led_on
            # bool is_power_on
            res = future.result()
            if res.cmd in ["on", "off"]:
                self._isProjOn = res.is_power_on
            elif res.cmd in ["led-on", "led-off"]:
                self._isLedOn = res.is_led_on
            self.get_logger().info('Projector status: %s\n' % (res.status))
            self.get_logger().info('Projector message: %s\n' % (res.msg))

            # TODO: send gui a message: status
        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished async call....')

    def pi_future_callback(self, future):
        try:
            # TODO: update --> self.printerController[i]._queue_index
            # ---
            # Metadata meta
            # int32 id
            # string cmd
            # int32 err
            # string[] videos
            # string[] queue
            # string msg
            # string status
            # bool is_video_on
            res = future.result()
            if res.cmd == "get-videos":
                if res.videos != None and len(res.videos) > 0:
                    self._pi_videos = res.videos
                else:
                    self.get_logger().warning(
                        "[pi_future_callback]: No videos returned")

            if res.cmd == "get-queue":
                if res.videos != None and len(res.queue) > 0:
                    self._pi_queue = res.queue
                else:
                    self.get_logger().warning(
                        "[pi_future_callback]: No queue returned")
            if res.cmd in ["play", "stop"]:
                self._isVideoOn = res.is_video_on

            self.get_logger().info('Pi Video status %s' % (res.status))
            self.get_logger().info('res.is_led_on %s' % (res.msg))

            # TODO send status message to GUI

        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished async call....')

    def motor_print_future_callback(self, future):
        try:
            # ---
            # Metadata meta
            # int32 id
            # string cmd
            # int32 err
            # string msg
            # string status
            # int32 set_speed
            res = future.result()
            if res.cmd == "on":
                self._motor._speed = res.set_spped
            else:
                self._motor._speed = 0

            self.get_logger().info('Print Motor status %s' % (res.status))

        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished async call....')

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
    def reset_playing_queue(self):
        for item in self._pi_queue:
            self._playing_queue.put(item)
        self._isPlayingQueueEmpty = False
