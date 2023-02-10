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
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.frame_processor import FrameProcessor
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .dense_optical_flow import DenseOpticalFlow

class DenseOpticalFlowProviderNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_dense_optical_flow_provider')

    # setup services, publishers and subscribers
    self.sub_grey_frame = self.create_subscription(Image, 'sky360/frames/grey', self.grey_frame_callback, subscriber_qos_profile)
    self.pub_dense_optical_flow_frame = self.create_publisher(Image, 'sky360/frames/dense_optical_flow', publisher_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def grey_frame_callback(self, msg_frame:Image):

    if msg_frame != None:

      try:
        frame_grey = self.br.imgmsg_to_cv2(msg_frame)

        optical_flow_frame = self.frame_processor.process_optical_flow(self.dense_optical_flow, frame_grey, None)
      
        frame_optical_flow_msg = self.br.cv2_to_imgmsg(optical_flow_frame, encoding="bgr8")
        frame_optical_flow_msg.header = msg_frame.header

        self.pub_dense_optical_flow_frame.publish(frame_optical_flow_msg)
      except Exception as e:
        self.get_logger().error(f"Exception during the dense optical flow. Error: {e}.")
        self.get_logger().error(tb.format_exc())

  def config_list(self) -> List[str]:
    return ['dense_optical_flow_h', 'dense_optical_flow_w', 'dense_optical_cuda_enable']

  def validate_config(self) -> bool:
    valid = True

    if self.app_configuration['dense_optical_flow_h'] == None:
      self.get_logger().error('The dense_optical_flow_h config entry is null')
      valid = False
      
    if self.app_configuration['dense_optical_flow_w'] == None:
      self.get_logger().error('The dense_optical_flow_w config entry is null')
      valid = False

    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()

    self.frame_processor = FrameProcessor.Select(self.app_configuration, 'dense_optical_cuda_enable')
    self.dense_optical_flow = DenseOpticalFlow.Select(self.app_configuration)
    

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = qos_profile_sensor_data #get_topic_subscriber_qos_profile()
  publisher_qos_profile = qos_profile_sensor_data #get_topic_publisher_qos_profile()

  node = DenseOpticalFlowProviderNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()

if __name__ == '__main__':
  main()