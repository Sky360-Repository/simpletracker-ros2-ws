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
import rclpy
from rclpy.qos import QoSProfile, QoSPresetProfiles, qos_profile_sensor_data
from typing import List
from std_msgs.msg import String
from simple_tracker_interfaces.msg import TrackingState
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile

class PrometheusNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_prometheus')

    # setup services, publishers and subscribers    
    self.sub_status = self.create_subscription(TrackingState, 'sky360/tracker/tracking_state', self.state_callback, subscriber_qos_profile)

    self.pub_tracking_state_json = self.create_publisher(String, 'sky360/tracker/tracking_state/json', publisher_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def state_callback(self, msg_tracking_state:TrackingState):
    string_msg = String()
    string_msg.data = f"{{\"trackable\":{msg_tracking_state.trackable}, \"alive\":{msg_tracking_state.alive}, \"started\":{msg_tracking_state.started}, \"ended\":{msg_tracking_state.ended}}}"
    self.pub_tracking_state_json.publish(string_msg)

  def config_list(self) -> List[str]:
    return []

  def validate_config(self) -> bool:
    valid = True
    return valid

  def on_config_loaded(self, init: bool):
    if init:
      pass

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = qos_profile_sensor_data #get_topic_subscriber_qos_profile()
  publisher_qos_profile = qos_profile_sensor_data #get_topic_publisher_qos_profile()

  node = PrometheusNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()