import rclpy
from rclpy.node import Node
from simple_tracker_interfaces.srv import ConfigEntry
from .app_settings import AppSettings

class SimpleTrackerConfigurationNode(Node):
        
        def __init__(self):

                super().__init__('simple_tracker_configuration')
                self.settings = AppSettings.Get()
                self.srv = self.create_service(ConfigEntry, 'sky360_configuration', self.get_config_callback)
                self.get_logger().info(f'SimpleTrackerConfigurationNode up and running.')

        def get_config_callback(self, request, response):

                self.get_logger().info(f'Requesting setting for [{request.key}].')

                value = self.settings[request.key]
                response.type = type(value).__name__
                response.value = str(value)

                return response