# Original work Copyright (c) 2022 Sky360
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile
from rclpy.node import Node
from simple_tracker_interfaces.msg import ConfigItem, ConfigEntryUpdatedArray
from simple_tracker_interfaces.srv import ConfigEntryUpdate, ConfigEntry, ConfigEntryArray
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.config_entry_convertor import ConfigEntryConvertor
from simple_tracker_shared.qos_profiles import get_config_publisher_qos_profile
from .app_settings import AppSettings

class SimpleTrackerConfigurationNode(Node):

    def __init__(self, publisher_qos_profile: QoSProfile):

        super().__init__('sky360_configuration')

        # Mike: Not sure of these things are thread safe, but this is just a proof of concept etc
        self.settings = AppSettings.Get(self)

        self.config_service = self.create_service(ConfigEntry, 'sky360/config/entry', 
            self.get_config_callback)
        self.config_service = self.create_service(ConfigEntryArray, 'sky360/config/entries', 
            self.get_config_array_callback)
        self.config_change_service = self.create_service(ConfigEntryUpdate, 'sky360/config/entry/update', 
            self.get_config_update_callback)
        self.config_change_publisher = self.create_publisher(ConfigEntryUpdatedArray, 'sky360/config/updated', 10)#, publisher_qos_profile)

        self.get_logger().info(f'{self.get_name()} node is up and running.')

    def get_config_callback(self, request, response):

        #self.get_logger().info(f'Requesting config key: [{request.key}].')

        value = self.settings[request.key]

        item = ConfigItem()

        item.key = request.key
        item.type = type(value).__name__
        item.value = str(value)

        response.entry = item

        return response

    def get_config_array_callback(self, request, response):

        #self.get_logger().info(f'Requesting config keys: [{request.keys}].')

        config_items = []

        for key in request.keys:
            value = self.settings[key]

            item = ConfigItem()
            item.key = key
            if value is not None:
                item.type = type(value).__name__
                item.value = str(value)
            else:
                item.type = 'null'
                item.value = ''

            config_items.append(item)

        response.entries = config_items

        return response

    def get_config_update_callback(self, request, response):

        self.get_logger().info(f'Updating config.')

        updated = False
        message = 'NoChange/Unknown/Failed'
        message_inner = ''
        keys = []

        for config_entry in request.entries:

            type_validated = True

            if config_entry.key in self.settings:
                previous_value = self.settings[config_entry.key]
                updated_value = ConfigEntryConvertor.Convert(config_entry.type, config_entry.value)
                self.get_logger().info(f'Updating known config {config_entry.key} to {config_entry.value}.')

                if previous_value is not None:
                    if type(previous_value).__name__ != type(updated_value).__name__:
                        type_validated = False
                        message_inner += f'Updating [{config_entry.key}] failed. Type mismatch {type(previous_value).__name__} != {type(updated_value).__name__}.'
                        self.get_logger().warn(message_inner)
                else:
                    self.settings[config_entry.key] = ConfigEntryConvertor.Convert(config_entry.type, config_entry.value)
                    self.get_logger().info(f'Updating config {config_entry.key}.')
                    message_inner += f'Updated: {config_entry.key},'
                    updated = True

                if type_validated:
                    keys.append(config_entry.key)
                    previous_value = self.settings[config_entry.key]
                    updated_value = ConfigEntryConvertor.Convert(config_entry.type, config_entry.value)

                    if previous_value != updated_value:
                        self.settings[config_entry.key] = ConfigEntryConvertor.Convert(config_entry.type, config_entry.value)
                        self.get_logger().info(f'Updating config {config_entry.key}.')
                        message_inner += f'Updated: {config_entry.key},'
                        updated = True
                    else:
                        self.get_logger().info(f'Ignoring config {config_entry.key} as values have not changed.')
            else:
                self.get_logger().warn(f'Unknown config {config_entry.key} key.')

        if updated:
            msg = ConfigEntryUpdatedArray()
            msg.keys = keys
            self.config_change_publisher.publish(msg)
            message = f'Success - {message_inner}'

        response.success = updated
        response.message = message
        return response


def main(args=None):

    #publisher_callbacks = PublisherEventCallbacks(incompatible_qos=lambda event: get_logger('Talker').info(str(event)))
    #subscription_callbacks = SubscriptionEventCallbacks(incompatible_qos=lambda event: get_logger('Listener').info(str(event)))
    #subscription_callbacks = SubscriptionEventCallbacks(message_lost=self._message_lost_event_callback)
    #subscription_callbacks = SubscriptionEventCallbacks(liveliness=lambda event: get_logger('Listener').info(str(event)))

    rclpy.init(args=args)

    publisher_qos_profile = get_config_publisher_qos_profile()

    node = SimpleTrackerConfigurationNode(publisher_qos_profile)

    runner = NodeRunner(node)
    runner.run()


if __name__ == '__main__':
  main()