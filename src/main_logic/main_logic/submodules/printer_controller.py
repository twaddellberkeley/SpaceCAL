import time
from datetime import datetime
from threading import Thread
import threading
from functools import partial
from queue import Queue

from interfaces.srv import GuiDisplay, GuiInput, Projector, Video, MotorSrv
import asyncio

AWAIT_TIME = 5.0

import rclpy
from rclpy.node import Node

STATE_STOPPED = 0x00
STATE_READY = 0x01
STATE_MOVING = 0x10
STATE_PRINTING = 0x100


class Motor():
    def __init__(self, id):
        self._id = id
        self._status = "off"
        self._speed = 0
        self._isMoving = False


class PrinterController(Node):
    def __init__(self, id, logger=None):
        super().__init__("printer_controller_node_" + str(id))
        self._id = id
        self._status = "off"
        self._isProjOn = False
        self._isLedOn = False
        self._isVideoOn = False
        self._isPlayingQueueEmpty = True
        self._motor = Motor(id)
        self._pi_videos = ['truss.mp4','OP_Oring.mp4', 'HV_Sphere.mp4', 'dot.mp4', 'PEG_Sphere.mp4', 'PEG_Starship.mp4', 'LV_Benchy.mp4', 'dogbone_oath_720.mp4', 'dogbone_oath_crop.mp4', 'PEG_Flex.mp4', 'LV_Thinker.mp4', 'HV_Flex.mp4', 'PEG_Tetra.mp4', 'OP_Chain.mp4', '242N_Flex.mp4', 'LV_Flex.mp4', 'thinker_mod.mp4', 'alignment_lines_650.mp4', 'test.mp4', '3test.mp4', 'OP_Screwdriver.mp4', 'alignment_lines_630.mp4', 'video.mp4', 'OP_Tube.mp4', 'out.mp4', 'cylinder.mp4', 'LV_Tetra.mp4', 'LV_OC.mp4', 'OP_Hinge.mp4', 'dogbone_oath.mp4', 'alignment_lines_598.mp4', 'PEG_Shuttle.mp4', 'Gelma.mp4', 'PEG_Spike.mp4', 'PEG_SLS.mp4', 'thinker_mod_720.mp4', 'smol_thinker.mp4', 'PEG_OC.mp4', '242N_Oring.mp4', '242N_Comp.mp4', 'HV_Spike.mp4', 'truss.mp4', '2test.mp4', 'LV_Spike.mp4', 'video24dps.mp4', 'HV_OC.mp4', 'LV_Sphere.mp4']
        self._pi_queue = ['OP_Oring.mp4', 'HV_Sphere.mp4', 'dot.mp4', 'PEG_Sphere.mp4', 'PEG_Starship.mp4', 'LV_Benchy.mp4', 'dogbone_oath_720.mp4']
        self._playing_queue = Queue()
        self.logger = logger
        

        ############## Create Client services ####################
        self.pi_cli = self.create_client(
            Video, "pi_video_srv_" + str(self._id))
        self.proj_cli = self.create_client(
            Projector, "projector_srv_" + str(self._id))
        self.motor_cli = self.create_client(
            MotorSrv, "print_motor_srv_" + str(self._id))
        self.gui_cli = self.create_client(
            GuiDisplay, "gui_display_srv")


    def send_pi_cmd(self, cmd):
        self.pi_controller_logic(cmd)

    def send_proj_cmd(self, cmd_tuple):
        self.proj_controller_logic(cmd_tuple)

    def send_motor_cmd(self, cmd):
        self.motor_controller_logic(cmd)

    ################################################################

    def pi_controller_logic(self, pi_cmd):
        # This function already knows the pi number to make request
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

        get_videos = "get-videos"
        get_queue = "get-queue"
        play_video = "play"
        stop = "stop-video"
        update = "update-queue"
        reset = "reset-queue"
        gui_queue = "queue"
        gui_videos = "videos"

        # this is to skip the "pi" in pi_cmd
        index = 1
        split_cmd = pi_cmd.split("-")
        req = Video.Request()
        gui_req = GuiDisplay.Request()
        req.id = self._id

        # Process the get command
        if split_cmd[index] == "get":
            # TODO: return vidoe or queue to gui
            if "queue" in split_cmd:
                # TODO: return queue to gui
                print(self._pi_queue)
                gui_req.pi_queue = self._pi_queue
                gui_req.printer_id = self._id
                gui_req.cmd = gui_queue
                self.gui_cli_req(gui_req)
                return
                # get info from controller in the future

            elif "videos" in split_cmd:
                # TODO: return videos to gui
                print(self._pi_videos)
                gui_req.pi_videos = self._pi_videos
                gui_req.printer_id = self._id
                gui_req.cmd = gui_videos
                self.gui_cli_req(gui_req)
                # self.pi_req("get-videos")
                return
            else:
                self.get_logger().error(
                    "[pi_control_logic]: Invalid Command")

        # Process the play command
        elif split_cmd[index] == "play":
            req.cmd = play_video

            # We get a name form the queue
            if split_cmd[index+1] == "queue":
                # Make sure playing queue is not empty
                self.get_logger().info("Requested to play video: HERE")
                if self._playing_queue.empty():
                    self._isPlayingQueueEmpty = True
                    self.get_logger().info("Queue is Empty")
                    print(self._pi_queue)
                    # TODO: stop printing since queue is done
                else:
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
                self.get_logger().info("This file does not exist: %s\n" % (split_cmd[index+1]))
            # print(req.file_name)

            self.get_logger().info("Requested to play video: %s\n" % (req.file_name))
        # Process the stop command
        elif split_cmd[index] == "stop":
            req.cmd = stop

        # Reset the queue
        elif split_cmd[index] == "reset":
            self.reset_playing_queue()
            # TODO: send confirmation to gui
            return

        # Command is not recognized
        else:
            self.get_logger().error(
                '[proj_controller_logic]: Command not recognized ')

        self.pi_req(req)

    def proj_controller_logic(self, proj_t):
        """This Function takes a command PROJ_CMD and implements the logic necessary to complete the comand
                - **proj-on-<#>**: turns on the projector number # and parks the DMD
                - **proj-off-<#>**: turns off the projector number # and parks the DMD
                - **proj-led-on-<#>**: turns the projector number #’s led on
                - **proj-led-off-<#>**: turns the projector number #’s led off
        """
        led_on = "led-on"
        led_off = "led-off"
        power_on = "on"
        power_off = "off"
        proj_cmd = proj_t[0]
        assert (proj_cmd != None)
        index = 1
        
        
        split_cmd = proj_cmd.split("-")
        req = Projector.Request()
        
        req.id = -1
        if proj_t[1] > 1:
            req.id = proj_t[1]

        # Turn on HDMI power
        if split_cmd[index] == "on":
            # Turn projector on
            req.cmd = power_on

        # Trurn HDMI power off
        elif split_cmd[index] == "off":
            # # Trun projector off
            # if self._isLedOn:
            #     # If the led is on turn it off first
            #     req.cmd = led_off
            #     self.proj_req(req)
            #     # wait to turn led off
            #     time.sleep(0.5)
            req.cmd = power_off

        # Handle LED
        elif split_cmd[index] == "led":
            # Turn LED on
            if split_cmd[index + 1] == "on":
                # Turn led on
                # if not self._isProjOn:
                #     # Verify projector is on else turn it on first
                #     req.cmd = power_on
                #     self.proj_req(req)
                #     # wait for projector to be turn on
                #     time.sleep(2)
                req.cmd = led_on
            elif split_cmd[index + 1] == "off":
                req.cmd = led_off
            # Command is not recognized
            else:
                self.get_logger().error(
                    '[proj_controller_logic]: Command not recognized ')
        else:
            self.get_logger().error(
                '[proj_controller_logic]: Command not recognized ')
        self.proj_req(req)

    def motor_controller_logic(self, motor_cmd):
        """ TODO: Description of this function 
            - **motor-on-<speed>-<#>**: turns motor **#** with **speed**
            - **motor-off-<#>:** turn motor # off
        """
        # TODO: add any logic if neccessary when turning motors on or off
        assert (motor_cmd != None)
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
        self.motor_req(req)

    ################################################################

    def pi_req(self, req):

        ############# temp variables for debug ###############
        time_to_wait = 4
        count = 0
        ######################################################

        while not self.pi_cli.wait_for_service(timeout_sec=.2):
            self.get_logger().info('service not available, waiting again...')
            if count > time_to_wait:
                return
            count += 1
        self.get_logger().warning(
            "[printer_controller]: Sending request cmd: %s" % (req.cmd))

        self.pi_future = self.pi_cli.call_async(req)
        self.pi_future.add_done_callback(partial(self.pi_req_callback))
        ################################################################
        self.get_logger().debug('Waiting on Pi async...')   # DEBUG message
    
    def pi_req_callback(self, future):
        gui_req = GuiDisplay.Request()
        gui_req.printer_id = self._id
        gui_req.cmd = "display+state"

        try:
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
            res = self.pi_future.result()
            if res.cmd == "get-videos":
                if res.videos != None and len(res.videos) > 0:
                    print(res.videos)
                    self._pi_videos = res.videos
                else:
                    self.get_logger().warning(
                        "[pi_future_callback]: No videos returned")

            if res.cmd == "get-queue":
                if res.queue != None and len(res.queue) > 0:
                    print(res.queue)
                    self._pi_queue = res.queue
                    self.reset_playing_queue()
                else:
                    self.get_logger().warning(
                        "[pi_future_callback]: No queue returned")

            if res.cmd in ["play", "stop"]:
                self._isVideoOn = res.is_video_on

                gui_req.display_name = "proj-video-" + str(self._id)
                gui_req.display_msg = res.video_playing
                self.gui_cli_req(gui_req)

            self.get_logger().info('Pi Video status %s' % (res.status))
            self.get_logger().info('res.is_led_on %s' % (res.msg))
            

        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished Pi async call....')

    def proj_req(self, req):
        ############# temp variables for debug ###############
        time_to_wait = 4
        count = 0
        ######################################################

        while not self.proj_cli.wait_for_service(timeout_sec=.2):
            self.get_logger().info('service not available, waiting again...')
            if count > time_to_wait:
                return
            count += 1
        self.get_logger().warning(
            "[printer_controller]: Sending request cmd: %s" % (req.cmd))
        
        self.proj_future = self.proj_cli.call_async(req)
        self.proj_future.add_done_callback(self.proj_req_callback)
        ################################################################
        self.get_logger().debug('Waiting on Projector async...')   # DEBUG message
        
    def proj_req_callback(self, future):
        gui_req = GuiDisplay.Request()
        gui_req.printer_id = self._id
        gui_req.cmd = "display+state"

        try:
            # Metadata meta
            # int32 id
            # string cmd
            # int32 err
            # string msg
            # string status
            # bool is_led_on
            # bool is_power_on
            res = self.proj_future.result()
            if res.cmd in ["on", "off"]:
                self._isProjOn = res.is_power_on
                gui_req.display_name = "proj-status-" + str(self._id)
                gui_req.display_msg = res.cmd
            elif res.cmd in ["led-on", "led-off"]:
                self._isLedOn = res.is_led_on
                gui_req.display_name = "proj-status-" + str(self._id)
                gui_req.display_msg = res.cmd

            gui_req.state = self.get_printer_status()
            self.gui_cli_req(gui_req)
            self.get_logger().info('Projector status: %s\n' % (res.status))
            self.get_logger().info('Projector message: %s\n' % (res.msg))

        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished Projector async call....')

    def motor_req(self, req):
        ############# temp variables for debug ###############
        time_to_wait = 4
        count = 0
        ######################################################

        while not self.motor_cli.wait_for_service(timeout_sec=.2):
            self.get_logger().info('service not available, waiting again...')
            if count > time_to_wait:
                return
            count += 1
        self.get_logger().warning(
            "[printer_controller]: Sending request cmd: %s" % (req.cmd))

        self.motor_future = self.motor_cli.call_async(req)
        self.motor_future.add_done_callback(partial(self.motor_req_callback))
        ################################################################
        self.get_logger().debug('Waiting on Motor async...')   # DEBUG message

    def motor_req_callback(self, future):
        gui_req = GuiDisplay.Request()
        gui_req.printer_id = self._id
        gui_req.cmd = "display+state"

        try:
            # ---
            # Metadata meta
            # int32 id
            # string cmd
            # int32 err
            # string msg
            # string status
            # int32 set_speed
            res = self.motor_future.result()
            if res.cmd == "on":
                self._motor._speed = res.set_speed
                self._motor._isMoving = True
            else:
                self._motor._speed = 0
                self._motor._isMoving = False

            gui_req.display_name = "printer-motor-" + str(self._id)
            gui_req.display_msg = str(self._motor._speed)
            gui_req.state = self.get_printer_status()
            self.gui_cli_req(gui_req)

            self.get_logger().info('Print Motor status %s' % (res.status))

        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished Motor async call....')
        
    def gui_cli_req(self, req):
        ############# temp variables for debug ###############
        time_to_wait = 4
        count = 0
        ######################################################

        while not self.gui_cli.wait_for_service(timeout_sec=.2):
            self.get_logger().info('service not available, waiting again...')
            if count > time_to_wait:
                return
            count += 1
        self.get_logger().warning(
            "[Sending to Gui]: Sending request cmd: %s" % (req.cmd))

        self.gui_future = self.gui_cli.call_async(req)
        # Add callback to receive response
        self.gui_future.add_done_callback(partial(self.gui_cli_req_callback))
        ################################################################
        self.get_logger().debug('Waiting on Gui async...')   # DEBUG message

    def gui_cli_req_callback(self, future):

        try:
            response = self.gui_future.result()
            self.get_logger().debug('Print Gui Display Result.msg: %s' % (response.msg))
            if response.err != 0:
                self.get_logger().error('[ERROR]: gui display response error %s' %
                                        (response.msg))
        except Exception as e:
            self.get_logger().error('ERROR: --- %r' % (e,))
        self.get_logger().info('finished Gui async call....')
            
    #********* Utility functions ***********#
    def reset_playing_queue(self):
        self.get_logger().info("Requested to reset queue")
        for item in self._pi_queue:
            self._playing_queue.put(item)
        self._isPlayingQueueEmpty = False

    def get_printer_status(self):
        if self._isLedOn and self._motor._isMoving and self._isVideoOn:
            return STATE_PRINTING
        elif self._motor._isMoving:
            if self._isVideoOn:
                return STATE_READY
        return STATE_STOPPED
















    # def proj_req(self, req):
    #     ############# temp variables for debug ###############
    #     time_to_wait = 4
    #     count = 0
    #     ######################################################

    #     while not self.proj_cli.wait_for_service(timeout_sec=.2):
    #         self.get_logger().info('service not available, waiting again...')
    #         if count > time_to_wait:
    #             return
    #         count += 1
    #     self.get_logger().warning(
    #         "[printer_controller]: Sending request cmd: %s" % (req.cmd))
        
    #     t = threading.Thread(target=self.proj_req, args=[req])

    #     future = self.proj_cli.call_async(req)
    #     future.add_done_callback(partial(self.proj_req_callback))
    #     rclpy.spin_until_future_complete(self, future)

    #     gui_req = GuiDisplay.Request()
    #     gui_req.printer_id = self._id
    #     gui_req.cmd = "display+state"

    #     try:
    #         # Metadata meta
    #         # int32 id
    #         # string cmd
    #         # int32 err
    #         # string msg
    #         # string status
    #         # bool is_led_on
    #         # bool is_power_on
    #         res = future.result()
    #         if res.cmd in ["on", "off"]:
    #             self._isProjOn = res.is_power_on
    #             gui_req.display_name = "proj-status-" + str(self._id)
    #             gui_req.display_msg = res.cmd
    #         elif res.cmd in ["led-on", "led-off"]:
    #             self._isLedOn = res.is_led_on
    #             gui_req.display_name = "proj-status-" + str(self._id)
    #             gui_req.display_msg = res.cmd

    #         gui_req.state = self.get_printer_status()
    #         self.gui_cli_req(gui_req)
    #         self.get_logger().info('Projector status: %s\n' % (res.status))
    #         self.get_logger().info('Projector message: %s\n' % (res.msg))

           

    #     except Exception as e:
    #         self.get_logger().error('ERROR: --- %r' % (e,))
    #     self.get_logger().info('finished async call....')