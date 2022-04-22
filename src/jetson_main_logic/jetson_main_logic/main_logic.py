

# This node takes input from the jetson Nano display and performs the required command.
# There are a total of 9 motors and 5 projectors that need to be control.

# Actions that need to be performed  (4 Motors )
##### Motors Level Control #######
# turn on the motors
# set the mottors home
# set the position of the motors
# pause current action
# kill the motors

# Motors Rotation Control #######  (5 Motors)
# Turn Motors on
# Set motor velocity
# Start motor rotation
# stop motor rotation
# kill motors


# Projector #######   (5 Projectors)
# Create a video queue
# Add video to queue
# Remove video from queue
# Rearrange queue
# Turn projector on
# Turn projector off
# Turn LED on
# Turn LED off
# Start playing video queue


#  We going to develop two diferent controllers that will allow us to control spaceCal as follows:
# A Level Controller -> This will controll the height of the plataform
# A CalPrint Controller -> This will control each individual print at any time

##### Level Controller ######
# This consist of 4 motors that need to move in harmony
# This will be a class Node that will implement the following comands
import imp
from urllib import response
import rclpy
import time
import threading
import rclpy
from rclpy.node import Node

from interfaces.msg import MotorData
from interfaces.srv import GuiSrv, MotorSrv, ProjectorSrv


class Motor:
    _speed = 0
    # _status is False if motor is off and True if on
    _status = False

    def __init__(self, id) -> None:
        _motor_id = id
        pass

    def set_speed(slef, speed):
        pass

    def set_status(self, status):
        pass

    def get_motor_id(self):
        return self._motor_id

    def get_speed(self):
        return self._speed

    def get_status(self):
        return self._status


class Projector:
    _status_LED = False
    # _video_tile holds the name of video currently loded to play
    _video_title = ""
    _status_video = False

    def __init__(self, id) -> None:
        _pjtr_id = id
        pass

    def set_status_LED(slef, status):
        pass

    def set_video_title(self, title):
        pass

    def set_status_video(slef, status):
        pass

    def get_status_LED(self):
        return self._status_LED

    def get_video_title(self):
        return self._video_title

    def get_status_video(self):
        return self._status_video


class LevelController(Node):
    def __init__(self) -> None:
        super().__init__('Level_Controller_Node')


class CalPrintController(Node):
    class SendRequest(threading.Thread):
        def __init__(self, threadID, cli, req, node):  # , fn):
            threading.Thread.__init__(self)
            self.threadID = threadID
            self.cli = cli
            self.req = req
            self.node = node

        def run(self):
            print("Running Thred id: " + self.threadID)
            self.process_request(self.cli, self.req)

        def process_request(self, cli, req):
            print("[process_request]: cmd_num " + str(req.cmd_num))
            self.future = cli.call_async(req)
            while rclpy.ok():
                # print("loop %d" % promise.done())
                if self.future.done():
                    print("loop %d" % self.future.done())
                    try:
                        if self.threadID == 'motor':
                            self.node._motor_res = self.future.result()
                            print('[motor process_request]: succesfull: %d' %
                                  self.node._motor_res.ok)
                        elif self.threadID == 'projector':
                            self.node._projector_res = self.future.result()
                            print('[projector process_request]: succesfull: %d' %
                                  self.node._projector_res.ok)
                    except Exception as e:
                        self.node.get_logger().info(
                            'Service call failed %r' % (e,))
                    else:
                        self.node.get_logger().info(
                            'Commad was successful!!')
                    break

    def __init__(self, id, node) -> None:
        super().__init__('CalPrint_Controller_Node_' + str(id))
        self._motor = Motor(id)
        self._projector = Motor(id)
        self._node = node

        # Define ros service Clients
        self._motor_cli = self._node.create_client(
            MotorSrv, 'motor_command_' + str(id))
        self._projector_cli = self._node.create_client(
            ProjectorSrv, 'projector_command_' + str(id))

        # Wait for motor service to be avalible
        while not self._motor_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('motor service not available, waiting again...')
        self._motor_req = MotorSrv.Request()
        self._motor_res = MotorSrv.Response()

        # Wait for projector service to be avaliable
        while not self._projector_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('projector service not available, waiting again...')
        self._projector_req = ProjectorSrv.Request()
        self._projector_res = ProjectorSrv.Response()

    def send_motor_req(self):
        self._motor_req = MotorSrv.Request()
        ## TODO: Temporary numbers ###########
        self._motor_req.cmd_num = 23
        self._motor_req.value = 49
        ### ####################################
        self._motor_res = MotorSrv.Response()
        self.motorThread = self.SendRequest(
            'motor', self._motor_cli, self._motor_req, self)  # , self.process_request)
        self.motorThread.start()

    def send_projector_req(self):
        self._projector_req = ProjectorSrv.Request()
        ## TODO: Temporary numbers ###########
        self._projector_req.cmd_num = 2
        self._projector_req.value = 9
        ### ####################################
        self._projector_res = ProjectorSrv.Response()
        self.motorThread = self.SendRequest(
            'projector', self._projector_cli, self._projector_req, self)  # , self.process_request)
        self.motorThread.start()


class ManiLogicController(Node):
    def __init__(self) -> None:
        super().__init__('Main_Logic_Controller_Node')
        self.srv = self.create_service(
            GuiSrv, 'gui_command', self.gui_command_callback)
        self.controller = CalPrintController(1, self)
        self.timer = self.create_timer(5, self.test_motor)
        self.timer2 = self.create_timer(6, self.test_projector)
        print("finished init fun")

    def gui_command_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

    def test_motor(self):
        print("[MainLogic]: response: %d" % self.controller._motor_res.ok)
        # self.controller.motorThread.start()
        self.controller.send_motor_req()

    def test_projector(self):
        print("Projector: %d" % self.controller._projector_res.ok)
        print("Motor: %d" % self.controller._motor_res.ok)


def main():
    print('Hi from jetson_main_logic.')
    rclpy.init()

    main_logic = ManiLogicController()

    rclpy.spin(main_logic)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
