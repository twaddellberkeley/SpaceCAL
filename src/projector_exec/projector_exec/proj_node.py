
import time
import os
import time
import subprocess
import multiprocessing
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
        
        res = self.process_cmd(request, response)

        self.get_logger().info('Service request has been proccessed at projector: %d ' % (self.proj_num))
        return res

    def process_cmd(self, req, res):
        msg = ""
        if req.cmd == "on":
            if subprocess.run(["vcgencmd", "display_power", "1"]).returncode != 0:
                self.get_logger().error("Could not turn HDMI Power On")
            res.status = "HDMI-Power-On"
            res.is_power_on = True
            msg = "Turned HDMI Power On"
        elif req.cmd == "off":
            if subprocess.run(["vcgencmd", "display_power", "0"]).returncode != 0:
                self.get_logger().error("Could not turn HDMI Power Off")
            res.status = "HDMI-Power-Off"
            res.is_power_on = False
            msg = "Turned HDMI Power Off"
        elif req.cmd == "led-on":
            self.get_logger().info("Turning LED On")
            subprocess.run("ledOn")
            res.is_led_on = True
            res.status = "LED-On"
            msg = "Turned LED On"
        elif req.cmd == "led-off":
            self.get_logger().info("Turning LED Off")
            subprocess.run("ledZero")
            res.status = "LED-Off"
            res.is_led_on = False
            msg = "Turned LED On"
        else:
            msg = "DID NOT FIND COMMAND"
        res.err = 0
        res.msg = "Projector " + str(self.proj_num) + " " + msg + " succesfully"
        return res
            # Turn the hdmi power off



def main(args=None):
    rclpy.init(args=args)
    service = ProjectorNode()
    print('Hi from projector_exec %d', service.proj_num)

    rclpy.spin(service)

    rclpy.shutdown()



if __name__ == '__main__':
    main()
