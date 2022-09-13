import sys
from simple_tracker_interfaces.srv import ConfigEntryArray
import rclpy
from rclpy.node import Node

class ConfigurationsClientAsync(Node):

    def __init__(self):
        super().__init__('configurations_client_async')
        self.client = self.create_client(ConfigEntryArray, 'sky360/config_array')
        self.get_logger().info('created config service client...')

        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('configurations service not available, waiting again...')
        self.request = ConfigEntryArray.Request()

    def send_request(self, keys):
        self.request.keys = keys
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()