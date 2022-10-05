

import time
from interfaces.srv import Video
import rclpy
from rclpy.node import Node

from .submodules.video_utils import kill_me


class VideoNode(Node):
    

    def __init__(self):
        super().__init__('pi_video_node_0')
        self.proj_srv = self.create_service(Video, 'pi_video_srv_0', self.pi_video_exec_callback)
       

    def pi_video_exec_callback(self, request, response):
        assert type(request.cmd) == type(""), "cmd is not a string"
        self.get_logger().info('Incoming request\ncmd: %s ' % (request.cmd))
        time.sleep(5)
        response.err = 0
        response.msg = "Video 0 executed succesfully"
        response.status = request.cmd.split("-")[-1]
        response.is_video_on = True
        response.is_led_on = True
        self.get_logger().info('Finished request from Video\ncmd: %s ' % (request.cmd))
        return response


def main(args=None):
    print('Hi from pi_video_controller 0.')
    rclpy.init(args=args)

    service = VideoNode()

    rclpy.spin(service)

    rclpy.shutdown()



if __name__ == '__main__':
    main()
