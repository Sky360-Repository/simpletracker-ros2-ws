import rclpy
from rclpy.node import Node
from simple_tracker_interfaces.msg import ConfigChangeNotification
from simple_tracker_interfaces.srv import ConfigEntryUpdate
from simple_tracker_interfaces.srv import ConfigEntry
from .app_settings import AppSettings
from .config_entry_convertor import ConfigEntryConvertor

class SimpleTrackerConfigurationNode(Node):
        
        def __init__(self):

                super().__init__('simple_tracker_configuration')

                # Mike: Not sure of these things are thread safe, but this is just a proof of concept etc
                self.settings = AppSettings.Get()

                self.config_service = self.create_service(ConfigEntry, 'sky360_config', self.get_config_callback)
                self.config_change_service = self.create_service(ConfigEntryUpdate, 'sky360_config_update', self.get_config_change_callback)
                self.config_change_publisher = self.create_publisher(ConfigChangeNotification, 'sky360_config_update_notifier', 10)                
                
                self.get_logger().info(f'simple_tracker_configuration up and running.')

        def get_config_callback(self, request, response):

                self.get_logger().info(f'Requesting config key: [{request.key}].')

                value = self.settings[request.key]
                response.type = type(value).__name__
                response.value = str(value)

                return response

        def get_config_change_callback(self, request, response):

                self.get_logger().info(f'Updating config key [{request.key}].')

                previous_value = self.settings[request.key]
                updated_value = ConfigEntryConvertor.Convert(request.type, request.value)
                updated = False

                if previous_value is None:
                        self.settings[request.key] = ConfigEntryConvertor.Convert(request.type, request.value)
                        updated = True

                else:
                        if previous_value != updated_value:
                                self.settings[request.key] = ConfigEntryConvertor.Convert(request.type, request.value)
                                updated = True

                if updated:
                        self.config_change_publisher.publish(request.key)

                response.success = updated
                return response