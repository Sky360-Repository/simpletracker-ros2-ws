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
from rclpy.qos import QoSProfile
from typing import List
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.frame_processor import FrameProcessor
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .background_subtractor import BackgroundSubtractor

class BackgroundSubtractionProviderNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_background_subtraction_provider')

    # setup services, publishers and subscribers
    self.sub_grey_frame = self.create_subscription(Image, 'sky360/frames/grey', self.grey_frame_callback, 10)#, subscriber_qos_profile)
    self.pub_foreground_mask_frame = self.create_publisher(Image, 'sky360/frames/foreground_mask', 10)#, publisher_qos_profile)
    self.pub_masked_background_frame = self.create_publisher(Image, 'sky360/frames/masked_background', 10)#, publisher_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def grey_frame_callback(self, msg_frame:Image):

    if msg_frame != None:

      try:
        frame_grey = self.br.imgmsg_to_cv2(msg_frame)
        frame_foreground_mask, frame_masked_background = self.frame_processor.process_bg_subtraction(
          self.background_subtractor, frame_grey, None)
        
        frame_foreground_mask_msg = self.br.cv2_to_imgmsg(frame_foreground_mask, msg_frame.encoding)
        frame_foreground_mask_msg.header = msg_frame.header
        
        frame_masked_background_msg = self.br.cv2_to_imgmsg(frame_masked_background, msg_frame.encoding)
        frame_masked_background_msg.header = msg_frame.header
        
        self.pub_foreground_mask_frame.publish(frame_foreground_mask_msg)
        self.pub_masked_background_frame.publish(frame_masked_background_msg)
      except Exception as e:
        self.get_logger().error(f"Exception during the background subtraction provider. Error: {e}.")
        self.get_logger().error(tb.format_exc())


  def config_list(self) -> List[str]:
    return ['background_subtractor_sensitivity', 'background_subtractor_type', 'background_subtractor_learning_rate', 'background_subtractor_cuda_enable',
    'frame_provider_resize_dimension_h', 'frame_provider_resize_dimension_w']

  def validate_config(self) -> bool:
    valid = True

    background_subtractor_type = self.app_configuration['background_subtractor_type']
    supported_bgsubtractors = {'KNN', 'MOG', 'MOG2', 'BGS_FD', 'BGS_SFD', 'BGS_WMM', 'BGS_WMV', 'BGS_ABL', 'BGS_ASBL', 'BGS_MOG2', 'BGS_PBAS', 'BGS_SD', 'BGS_SuBSENSE', 'BGS_LOBSTER', 'BGS_PAWCS', 'BGS_TP', 'BGS_VB', 'BGS_CB', 'SKY_WMV', 'SKY_VIBE'}
    supported_cuda_bgsubtractors = {'MOG2_CUDA', 'MOG_CUDA'}
    supported = False
    if self.app_configuration['background_subtractor_cuda_enable']:
        supported = background_subtractor_type in supported_cuda_bgsubtractors
    else:
        supported = background_subtractor_type in supported_bgsubtractors

    if supported == False:
      self.get_logger().error(f"Unknown background subtractor type ({background_subtractor_type}) when cuda-enabled: {self.app_configuration['background_subtractor_cuda_enable']}.")
      valid = False

    if self.app_configuration['background_subtractor_sensitivity'] == None:
      self.get_logger().error('The background_subtractor_sensitivity config entry is null')
      valid = False
      
    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()

    self.frame_processor = FrameProcessor.Select(self.app_configuration, 'background_subtractor_cuda_enable')
    self.background_subtractor = BackgroundSubtractor.Select(self.app_configuration)

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = BackgroundSubtractionProviderNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()

if __name__ == '__main__':
  main()