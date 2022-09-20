import sys
from sensor_msgs.msg import Image
from simple_tracker_interfaces.srv import Mask
import rclpy
from rclpy.node import Node

class MaskClientAsync(Node):

    def __init__(self):
        super().__init__('mask_client_async')
        self.client = self.create_client(Mask, 'sky360/mask/image/v1')
        self.get_logger().info('created mask service client...')
        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('mask service not available, waiting again...')
        self.request = Mask.Request()

    def send_request(self, path):
        self.request.path = path
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()