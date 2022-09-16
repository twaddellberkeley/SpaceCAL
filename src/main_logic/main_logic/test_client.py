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



def main(args=None):
    rclpy.init(args=args)

    test_client = TestClient()
    response = test_client.send_request(sys.argv[1])
    test_client.get_logger().info(
        'Result of gui input: %s with error = %d ' %
        (sys.argv[1], response.err))

    test_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
