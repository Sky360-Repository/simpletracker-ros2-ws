import rclpy
from rclpy.node import Node
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from simple_tracker_interfaces.msg import ConfigItem
from simple_tracker_interfaces.srv import ConfigEntryUpdate
from simple_tracker_interfaces.srv import ConfigEntry
from simple_tracker_interfaces.srv import ConfigEntryArray
from .app_settings import AppSettings
from .config_entry_convertor import ConfigEntryConvertor

class SimpleTrackerConfigurationNode(Node):
        
        def __init__(self):

                super().__init__('simple_tracker_configuration')

                # Mike: Not sure of these things are thread safe, but this is just a proof of concept etc
                self.settings = AppSettings.Get()

                self.config_service = self.create_service(ConfigEntry, 'sky360/config/entry/v1', self.get_config_callback)
                self.config_service = self.create_service(ConfigEntryArray, 'sky360/config/entries/v1', self.get_config_array_callback)
                self.config_change_service = self.create_service(ConfigEntryUpdate, 'sky360/config/entry/update/v1', self.get_config_update_callback)
                self.config_change_publisher = self.create_publisher(ConfigEntryUpdatedArray, 'sky360/config/updated/v1', 10)
                
                self.get_logger().info(f'simple_tracker_configuration up and running.')

        def get_config_callback(self, request, response):

                self.get_logger().info(f'Requesting config key: [{request.key}].')

                value = self.settings[request.key]

                item = ConfigItem()

                item.key = request.key
                item.type = type(value).__name__
                item.value = str(value)

                response.entry = item

                return response

        def get_config_array_callback(self, request, response):

                self.get_logger().info(f'Requesting config keys: [{request.keys}].')

                config_items = []

                for key in request.keys:
                        value = self.settings[key]

                        item = ConfigItem()
                        item.key = key
                        item.type = type(value).__name__
                        item.value = str(value)                        

                        config_items.append(item)

                response.entries = config_items

                return response

        def get_config_update_callback(self, request, response):

                self.get_logger().info(f'Updating config.')

                updated = False
                validated = True
                message = 'Success - '
                keys = []

                for config_entry in request.entries:
                        previous_value = self.settings[config_entry.key]
                        updated_value = ConfigEntryConvertor.Convert(config_entry.type, config_entry.value)

                        if previous_value is not None:
                                if type(previous_value).__name__ != type(updated_value).__name__:
                                        validated = False
                                        message += f'Updating [{config_entry.key}] failed. Type mismatch {type(previous_value).__name__} != {type(updated_value).__name__}.'

                if validated:
                        for config_entry in request.entries:
                                keys.append(config_entry.key)
                                previous_value = self.settings[config_entry.key]
                                updated_value = ConfigEntryConvertor.Convert(config_entry.type, config_entry.value)

                                if previous_value is None:
                                        self.settings[config_entry.key] = ConfigEntryConvertor.Convert(config_entry.type, config_entry.value)
                                        self.get_logger().info(f'Updating config {config_entry.key}.')
                                        message += f'Updated: {config_entry.key},'
                                        updated = True
                                else:
                                        if previous_value != updated_value:
                                                self.settings[config_entry.key] = ConfigEntryConvertor.Convert(config_entry.type, config_entry.value)
                                                self.get_logger().info(f'Updating config {config_entry.key}.')
                                                message += f'Updated: {config_entry.key},'
                                                updated = True

                if updated:
                        msg = ConfigEntryUpdatedArray()
                        msg.keys = keys
                        self.config_change_publisher.publish(msg)

                response.success = updated
                response.message = message
                return response