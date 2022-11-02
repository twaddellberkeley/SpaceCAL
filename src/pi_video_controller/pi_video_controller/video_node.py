
import os
import time
from interfaces.srv import Video
import rclpy
from rclpy.node import Node

from .submodules.video_utils import VlcControl, VIDEO_DIR, QUEUE_VIDEO_FILE

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
        self.pi_videos = self.get_pi_videos()
        self.pi_queue = self.get_pi_queue()
        # Create a service to recieve pi requests to this service name with PI_NUM
        self.proj_srv = self.create_service(Video, 'pi_video_srv_' + str(self.pi_num), self.pi_video_exec_callback)
       

    def pi_video_exec_callback(self, request, response):
        assert type(request.cmd) == type(""), "cmd is not a string"
        self.get_logger().info('\nService request recieved at PI %d\nCommand: %s ' % (self.pi_num, request.cmd))
        ############ TODO: Code Here ###################

        response = self.process_cmd(request, response)

        self.get_logger().info('Finished request from PI\ncmd: %s ' % (request.cmd))
        return response

    def process_cmd(self, req, res):

        if req.cmd == "play":
            self.vlc.playVideo(req.file_name)
            self.get_logger().info("video is playing")
            res.is_video_on = True
            res.status = "video playing"
        elif req.cmd == "stop-video":
            self.vlc.stopVideo()
            self.get_logger().info("video has stop playing")
            res.is_video_on = False
            res.status = "stopped"
        elif req.cmd == "get-videos":
            print(type(self.pi_videos))
            res.videos = self.pi_videos
        elif req.cmd == "get-queue":
            res.queue = self.pi_queue
        elif req.cmd =="update-queue":
            self.update_queue(req.update_queue)
        res.err = 0
        res.id = self.pi_num
        res.msg = "Projector " + str(self.pi_num) + " executed succesfully"
        return res
    
    def get_pi_videos(self):
        lst = []
        for file in os.listdir(VIDEO_DIR):
            if file.endswith(".mp4"):
                lst.append(file)
        return lst

    def update_queue(self, queue):
        assert queue != None, "Queue is None"
        if len(queue) < 1:
            return
        with open(QUEUE_VIDEO_FILE, 'w') as f:
            f.writelines('\n'.join(queue))

    def get_pi_queue(self):
        lst = []
        with open(QUEUE_VIDEO_FILE, 'r') as f:
            [lst.append(line) for line in f.readlines()]
        return lst
                

def main(args=None):
    rclpy.init(args=args)
    service = VideoNode()
    print('Hi from pi_video_controller %d.', service.pi_num)
    rclpy.spin(service)

    rclpy.shutdown()



if __name__ == '__main__':
    main()
