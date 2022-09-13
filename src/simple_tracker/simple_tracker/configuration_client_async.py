import sys
from simple_tracker_interfaces.srv import ConfigEntry
import rclpy
from rclpy.node import Node

class ConfigurationClientAsync(Node):

    def __init__(self):
        super().__init__('configuration_client_async')
        self.client = self.create_client(ConfigEntry, 'sky360/config/entry/v1')
        self.get_logger().info('created config service client...')

        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('configuration service not available, waiting again...')
        self.request = ConfigEntry.Request()

    def send_request(self, key):
        self.request.key = key
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()