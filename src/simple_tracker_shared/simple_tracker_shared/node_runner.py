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
from rclpy.node import Node

class NodeRunner():

    def __init__(self, node: Node):
        self.node = node

    def run(self):

        try:
            #self.node.get_logger().info(f'Running via from node runner')
            #rclpy.logging.get_logger('node_runner').info(f'Running node {self.node.get_name()} via the node runner')
            rclpy.spin(self.node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            #self.node.get_logger().info(f'Shutting down node from node runner')
            #rclpy.logging.get_logger('node_runner').info(f'Shutting down {self.node.get_name()} via the node runner')
            self.node.destroy_node()
            rclpy.try_shutdown()
