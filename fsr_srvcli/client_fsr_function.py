import sys

from fsr_interfaces.srv import Trigger
from fsr_interfaces.srv import StopSig
import rclpy
from rclpy.node import Node

class FsrClientAsync(Node):
    def __init__(self):
        super().__init__('fsr_client_async')
        self.cli = self.create_client(Trigger, 'trigger_recorder')
        #self.srv = self.create_service(StopSig, 'stop_signal', self.test_callback)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

    def send_request(self):
        self.req.duration = 5
        self.req.show_flag = False
        self.req.record_flag = True
        self.req.folder = '/home/hrg/ws_moveit/data/'
        self.req.labels = 'test_1'
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def test_callback(self, request, response):
        print('Value received', request.mean_val)
        response.success = True
        return response

def main():
    rclpy.init()

    fsr_client = FsrClientAsync()
    response = fsr_client.send_request()
    fsr_client.get_logger().info(
        'Result of trigger fsr recorder: %d' % response.success)

    fsr_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
