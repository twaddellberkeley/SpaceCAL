
import time
from interfaces.srv import Projector

import rclpy
from rclpy.node import Node

class ProjectorNode(Node):
    

    def __init__(self):
        super().__init__('projector_node')
        # Declare a parameter that can be set from a launch file to create distinct services: One for each projector.
        # The default value of the parameter is zero
        ###############################################################################################################
        self.declare_parameter("projector_number", 0)                      ####### ROS Parameter: "proj_number"   #####
        self.proj_num = self.get_parameter("projector_number").value       #######                                #####
        ###############################################################################################################

        # Create a service to recieve projector requests to this service name
        self.proj_srv = self.create_service(Projector, 'projector_srv_' + str(self.proj_num), self.projector_exec_callback)
       

    def projector_exec_callback(self, request, response):
        assert type(request.cmd) == type(""), "cmd is not a string"
        self.get_logger().info('\nService request recieved at Projector %d\nCommand: %s ' % (self.proj_num, request.cmd))
        time.sleep(5)
        response.err = 0
        response.msg = "Projector " + str(self.proj_num) + " executed succesfully"
        response.status = request.cmd.split("-")[-1]
        response.is_video_on = True
        response.is_led_on = True
        self.get_logger().info('Service request has been proccessed at projector: %d ' % (self.proj_num))
        return response


def main(args=None):
    rclpy.init(args=args)
    service = ProjectorNode()
    print('Hi from projector_exec %d', service.proj_num)

    rclpy.spin(service)

    rclpy.shutdown()



if __name__ == '__main__':
    main()
