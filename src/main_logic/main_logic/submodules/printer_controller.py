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
        self._motor = Motor(id)
        self._pi_videos = []
        self._pi_queue = Queue()
        self._queue_index = 0

    def send_motor_cmd(self, cmd):
        self.motor_controller_logic(cmd)

    def send_proj_cmd(self, cmd):
        self.proj_controller_logic(cmd)

    def send_pi_cmd(self, cmd):
        self.pi_controller_logic(cmd)

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
        if split_cmd[index] == "on":
            req.speed = int(split_cmd[index+1])
        elif split_cmd[index] == "off":
            req.speed = 0
        self.client_req(motor_cmd, client, req)

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
        req = Video.Request()
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
                        queue_index = self.printerController[i]._queue_index
                        # Verify that there is an item in the queue
                        self.is_valid_queue_index(queue_index)
                        # Get file name from queue
                        req.file_name = self.printerController[i]._pi_queue[queue_index]
                        # convert to singel play request
                        cmd = "pi-play-" + req.file_name + "-" + int(i)
                        # Send request
                        self.client_req(cmd, client, req)
                        # update queue index to the next video
                        self.printerController[i]._queue_index += 1
                    else:
                        # Get index from command
                        queue_index = int(split_cmd[index+1])
                        # Verify there is an item in the queue
                        self.is_valid_queue_index(queue_index)
                        # Get file name from queue
                        req.file_name = self.printerController[i]._pi_queue[queue_index]
                        # Get construct new command
                        cmd = "pi-play-" + req.file_name + "-" + int(i)
                        # Send command
                        self.client_req(cmd, client, req)

            # Send command to a single printer
            elif split_cmd[index+1] == "queue":
                # Get printer number
                printer_num = int(split_cmd[index+2])
                # Get next queue index from printer
                queue_index = self.printerController[printer_num]._queue_index
                # Get video from printer
                req.file_name = self.printerController[printer_num]._pi_queue[queue_index]
                self.client_req(cmd, client, req)
                self.printerController[printer_num]._queue_index += 1

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

    def client_req(self, cmd, client, req):
        # times to try to connect to server
        times_to_connect = 5

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

        # TODO: where do we recod metadata?
        count = 0
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
            # int32 id
            # int32 err
            # string msg
            # string status
            # bool is_led_on
            # bool is_power_on
            response = future.result()
            self.get_logger().info('Projector status %s' % (response.status))

        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished async call....')
        self.get_logger().info('Printer_1._isLedOn %s' %
                               ("True" if self.printerController[0]._isLedOn else "False"))

    def pi_future_callback(self, future):
        try:
            # TODO: update --> self.printerController[i]._queue_index
            res = future.result()
            self.get_logger().info('Pi Video status %s' % (res.status))
            # int32 id
            # int32 err
            # string[] videos
            # string[] queue
            # string msg
            # string status
            # bool is_video_on

            if res.videos != None and len(res.videos) > 0:
                self.printerController[res.id]._pi_videos = res.videos
            if res.videos != None and len(res.queue) > 0:
                [self.printerController[res.id]._pi_queue.put(
                    item) for item in res.queue]
            self.get_logger().info('res.is_led_on %s' %
                                   (res.msg))
        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished async call....')

    def motor_print_future_callback(self, future):
        try:
            # int32 id
            # int32 err
            # string msg
            # string status
            # int32 set_speed
            response = future.result()
            self.get_logger().info('Print Motor status %s' % (response.status))

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

    def is_proj_on(self, proj_num):
        assert type(proj_num) == type("")
        if proj_num == "all":
            for p in range(5):
                if not self.printerController[p]._isProjOn:
                    return False
            return True
        else:
            return self.printerController[int(proj_num)]._isProjOn

    def is_proj_led_on(self, proj_num):
        assert type(proj_num) == type("")
        if proj_num == "all":
            for p in range(5):
                if not self.printerController[p]._isLedOn:
                    return False
            return True
        else:
            return self.printerController[int(proj_num)]._isLedOn
