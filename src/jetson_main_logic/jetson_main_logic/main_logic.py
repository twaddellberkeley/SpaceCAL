

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
import rclpy
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from smbus2 import SMBus, i2c_msg
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
        def __init__(self, threadID, cli, req, fn):
            threading.Thread.__init__(self)
            self.threadID = threadID
            self.cli = cli
            self.req = req
            self.process_request = fn

        def run(self):
            print("Running Thred id: " + self.threadID)
            self.process_request(self.cli, self.req)

    def __init__(self, id) -> None:
        super().__init__('CalPrint_Controller_Node_' + str(id))
        self._motor = Motor(id)
        self._projector = Motor(id)

        # Define ros service Clients
        self._motor_cli = self.create_client(
            MotorSrv, 'motor_command_' + str(id))
        self._projector_cli = self.create_client(
            ProjectorSrv, 'projector_command_' + str(id))

        # Wait for motor service to be avalible
        while not self._motor_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('motor service not available, waiting again...')
        self._motor_req = MotorSrv.Request()

        # Wait for projector service to be avaliable
        while not self._projector_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('projector service not available, waiting again...')
        self._projector_req = ProjectorSrv.Request()
        self._motor_req.cmd_num = 23
        self._motor_req.value = 49

        self.motorThread = self.SendRequest(
            'motor', self._motor_cli, self._motor_req, self.process_request)

    def process_request(self, cli, req):
        print("[process_request]: cmd_num " + str(req.cmd_num))
        promise = cli.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if promise.done():
                try:
                    response = promise.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.get_logger().info(
                        'Commad was successful!!')
                break
        # while not exitFlag:
        #     queueLock.acquire()
        #        if not workQueue.empty():
        #             data = q.get()
        #             queueLock.release()
        #             print "%s processing %s" % (threadName, data)
        #         else:
        #             queueLock.release()
        #         time.sleep(1)


class ManiLogicController(Node):
    def __init__(self) -> None:
        super().__init__('Main_Logic_Controller_Node')
        self.srv = self.create_service(
            GuiSrv, 'gui_command', self.gui_command_callback)
        self.controller = CalPrintController(1)
        self.controller.motorThread.start()

    def gui_command_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main():
    print('Hi from jetson_main_logic.')
    rclpy.init()

    main_logic = ManiLogicController()

    rclpy.spin(main_logic)

    rclpy.shutdown()


if __name__ == '__main__':
    main()