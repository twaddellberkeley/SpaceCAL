

import time
from interfaces.srv import Video
import rclpy
from rclpy.node import Node

from .submodules.video_utils import VlcControl

class VideoNode(Node):
    

    def __init__(self):
        super().__init__('pi_video_node')
        # Declare a parameter that can be set from a launch file to create distinct services: One for each raspberry-pi.
        # The default value of the parameter is zero
        ####################################################################################################
        self.declare_parameter("pi_number", 0)                      ####### ROS Parameter: "pi_number" #####
        self.pi_num = self.get_parameter('pi_number').value         #######                            #####
        ####################################################################################################
        self.vlc = VlcControl(self.get_logger())

        # Create a service to recieve pi requests to this service name with PI_NUM
        self.proj_srv = self.create_service(Video, 'pi_video_srv_' + str(self.pi_num), self.pi_video_exec_callback)
       

    def pi_video_exec_callback(self, request, response):
        assert type(request.cmd) == type(""), "cmd is not a string"
        self.get_logger().info('\nService request recieved at PI %d\nCommand: %s ' % (self.pi_num, request.cmd))
        ############ TODO: Code Here ###################
        if request.cmd == "play":
            self.vlc.playVideo(request.file_name)
            self.get_logger().info("video is playing")
        else:
            self.vlc.stopVideo()
            self.get_logger().info("video has stop playing")

        response.err = 0
        response.msg = "Projector " + str(self.pi_num) + " executed succesfully"
        response.status = request.cmd.split("-")[-1]
        response.is_video_on = True
        response.is_led_on = True
        self.get_logger().info('Finished request from PI\ncmd: %s ' % (request.cmd))
        return response


def main(args=None):
    rclpy.init(args=args)
    service = VideoNode()
    print('Hi from pi_video_controller %d.', service.pi_num)
    rclpy.spin(service)

    rclpy.shutdown()



if __name__ == '__main__':
    main()
