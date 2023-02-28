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

import traceback as tb
import time
import rclpy
import asyncio
import tornado
import tornado.ioloop
import threading
from rclpy.qos import QoSProfile, QoSPresetProfiles, qos_profile_sensor_data
from std_msgs.msg import String
from simple_tracker_interfaces.msg import TrackingState
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from simple_tracker_shared.node_runner import NodeRunner

from .metrics_server import MetricsServer
from .handlers import PrometheusMetricsHandler

class PrometheusNode(object):
  
  instance = None

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile, node_name = "sky360_prometheus"):
    self.__class__.instance = self

    self.node = rclpy.create_node(node_name)

    self.sub_status = self.node.create_subscription(TrackingState, 'sky360/tracker/tracking_state', self.state_callback, subscriber_qos_profile)
    self.pub_tracking_state_json = self.node.create_publisher(String, 'sky360/tracker/tracking_state/json', publisher_qos_profile)

    self.metrics_server = MetricsServer(8082)

    # tornado event loop stuff
    self.event_loop = None
    asyncio.set_event_loop(asyncio.new_event_loop())
    self.event_loop = tornado.ioloop.IOLoop()
    self.metrics_server.start()

    # tornado event loop. all the web server and web socket stuff happens here
    threading.Thread(target = self.event_loop.start, daemon = True).start()

    self.logger = self.node._logger
    self.logger.info(f'{node_name} node is up and running.')

  def start(self):
    rclpy.spin(self.node)

  def destroy_node(self):
    self.node.destroy_node()

  def state_callback(self, msg_tracking_state:TrackingState):
    PrometheusMetricsHandler.state = msg_tracking_state
    string_msg = String()
    string_msg.data = f"{{\"trackable\":{msg_tracking_state.trackable}, \"alive\":{msg_tracking_state.alive}, \"started\":{msg_tracking_state.started}, \"ended\":{msg_tracking_state.ended}}}"
    self.pub_tracking_state_json.publish(string_msg)    

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = qos_profile_sensor_data #get_topic_subscriber_qos_profile()
  publisher_qos_profile = qos_profile_sensor_data #get_topic_publisher_qos_profile()

  node = PrometheusNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node.node)
  runner.run()

if __name__ == '__main__':
  main()