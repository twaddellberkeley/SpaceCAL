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


def test_gui_commands():
    test_client = TestClient()
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

    # test pi commands

    # test motor commands

    # test level commands

    # test custom commands
    test_client.destroy_node()

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
