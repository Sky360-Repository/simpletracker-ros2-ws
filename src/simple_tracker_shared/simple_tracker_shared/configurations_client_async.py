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
from simple_tracker_interfaces.srv import ConfigEntryArray
import rclpy
from rclpy.node import Node

class ConfigurationsClientAsync(Node):

    def __init__(self):
        super().__init__('configurations_client_async')
        self.client = self.create_client(ConfigEntryArray, 'sky360/config/entries/v1')
        self.get_logger().info('created config service client...')

        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('configurations service not available, waiting again...')
        self.request = ConfigEntryArray.Request()

    def send_request(self, keys):
        self.request.keys = keys
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()