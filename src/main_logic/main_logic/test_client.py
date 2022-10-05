import sys
from interfaces.srv import GuiInput

import time
import rclpy
from rclpy.node import Node


class TestClient(Node):

    def __init__(self):
        super().__init__('test_client_node')
        self.guiClient = self.create_client(
            GuiInput, 'gui_input_srv')
        while not self.guiClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GuiInput.Request()

    def send_request(self, cmd):
        self.req.cmd = cmd
        self.future = self.guiClient.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def test_proj_commads():
    test_client = TestClient()
    test_client.get_logger().info("\n*********** Starting Projector Test Commands **************\n")
    # test projector commands
    responseAll = test_client.send_request("proj-on-all")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("proj-on-all", responseAll.msg))
    
    response0 = test_client.send_request("proj-on-0")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("proj-on-0", response0.msg))

    response1 = test_client.send_request("proj-on-1")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("proj-on-1", response1.msg))

    response2 = test_client.send_request("proj-on-2")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("proj-on-2", response2.msg))
    
    response3 = test_client.send_request("proj-on-3")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("proj-on-3", response3.msg))
    
    response4 = test_client.send_request("proj-on-4")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("proj-on-4", response4.msg))
    test_client.destroy_node()

def test_pi_commads():
    # - **pi-get-videos-<#>**: gets all the videos available in the pi #
    # - **pi-get-queue-<#>**: get the video queue for pi #
    # - **pi-play-<videoName>-<#>**: play **videoName** from pi #
    # - **pi-stop-video-<#>**: stop playing video from pi #
    # - **pi-play-queue-<#>**: play video queue from pi #
    # - **pi-stop-queue-<#>**: stop video queue from pi # (this will exit the queue and wont remember where it stoped)
    # - **pi-pause-queue-<#>**: pauses the queue from pi # (This will stop the current print and get ready to play the next video.)
    test_client = TestClient()
    test_client.get_logger().info("\n*********** Starting Pi Video Test Commands **************\n")
    # test piector commands
    responseAll = test_client.send_request("pi-get-videos-all")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("pi-get-videos-all", responseAll.msg))
    
    response0 = test_client.send_request("pi-get-videos-0")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("pi-get-videos-0", response0.msg))

    response1 = test_client.send_request("pi-get-videos-1")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("pi-get-videos-1", response1.msg))

    response2 = test_client.send_request("pi-get-videos-2")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("pi-get-videos-2", response2.msg))
    
    response3 = test_client.send_request("pi-get-videos-3")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("pi-get-videos-3", response3.msg))
    
    response4 = test_client.send_request("pi-get-videos-4")
    test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            ("pi-get-videos-4", response4.msg))

    test_client.destroy_node()

def test_gui_commands():
    # test projector commands
    test_proj_commads()

    # test pi commands
    test_pi_commads()

    

    # test motor commands

    # test level commands

    # test custom commands
   

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        test_gui_commands()
    else:
        test_client = TestClient()
        response = test_client.send_request(sys.argv[1])
        test_client.get_logger().info(
            'Result of gui input: %s with msg = %s ' %
            (sys.argv[1], response.msg))
        test_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
