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
from simple_tracker_interfaces.srv import Mask

class MaskClientAsync(Node):

    def __init__(self):
        super().__init__('mask_client_async')
        self.client = self.create_client(Mask, 'sky360/mask/image/v1')
        self.get_logger().info('created mask service client...')
        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('mask service not available, waiting again...')
        self.request = Mask.Request()

    def send_request(self, file_name):
        self.request.file_name = file_name
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()