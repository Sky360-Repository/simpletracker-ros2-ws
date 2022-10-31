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

import sys
import rclpy
from rclpy.node import Node
from simple_tracker_interfaces.srv import ConfigEntryArray, ConfigEntryUpdate

class ConfigurationsClientAsync(Node):

    def __init__(self):
        super().__init__('configurations_client_async')
        self.get_config_client = self.create_client(ConfigEntryArray, 'sky360/config/entries/v1')
        self.update_config_client = self.create_client(ConfigEntryUpdate, 'sky360/config/entry/update/v1')
        self.get_logger().info('created config service client...')

        while not self.get_config_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('configuration get service not available, spinning ...')

        while not self.update_config_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('configuration update service not available, spinning ...')

        self.get_request = ConfigEntryArray.Request()
        self.update_request = ConfigEntryUpdate.Request()

    def send_get_config_request(self, keys):
        self.get_request.keys = keys
        self.future = self.get_config_client.call_async(self.get_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_update_config_request(self, config_array):
        self.update_request.entries = config_array
        self.future = self.update_config_client.call_async(self.update_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()