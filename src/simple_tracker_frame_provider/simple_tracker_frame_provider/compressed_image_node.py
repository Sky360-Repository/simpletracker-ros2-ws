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
import cv2
import rclpy
from rclpy.qos import QoSProfile, QoSPresetProfiles
from typing import List
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile

class CompressedImageNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_compressed_image')

    # setup services, publishers and subscribers
    self.sub_source = self.create_subscription(Image, 'sky360/compressed/source', self.source_callback, qos_profile=subscriber_qos_profile)
    self.pub_target = self.create_publisher(CompressedImage, 'sky360/compressed/target', 5)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def source_callback(self, msg_source:Image):

    if msg_source != None:

      try:

        source_frame = self.br.imgmsg_to_cv2(msg_source)

        compressed_msg: CompressedImage = CompressedImage()
        compressed_msg.header = msg_source.header
        compressed_msg.format = 'jpeg'
        compressed_msg.data = cv2.imencode('.jpg', source_frame)[1].tobytes()
        self.pub_target.publish(compressed_msg)

      except Exception as e:
        self.get_logger().error(f"Exception during compressed image provider. Error: {e}.")
        self.get_logger().error(tb.format_exc())

  def config_list(self) -> List[str]:
    return []

  def validate_config(self) -> bool:
    valid = True
    
    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.counter = 0
      self.br = CvBridge()

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = CompressedImageNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()