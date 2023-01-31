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

import datetime
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile
from typing import List
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.frame_processor import FrameProcessor
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .mask import Mask
from .mask_client_async import MaskClientAsync

class FrameProviderNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_frame_provider')

    # setup services, publishers and subscribers    
    self.sub_camera = self.create_subscription(Image, 'sky360/camera/original', self.camera_callback, 10)#, subscriber_qos_profile)
    self.pub_original_frame = self.create_publisher(Image, 'sky360/frames/original', 10)#, publisher_qos_profile)
    self.pub_masked_frame = self.create_publisher(Image, 'sky360/frames/masked', 10)#, publisher_qos_profile)
    self.pub_grey_frame = self.create_publisher(Image, 'sky360/frames/grey', 10)#, publisher_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_callback(self, msg_image:Image):

    if msg_image != None:

      self.counter += 1

      frame_original, frame_grey, frame_masked = self.frame_processor.process_for_frame_provider(self.mask, 
        self.br.imgmsg_to_cv2(msg_image), stream=None)

      frame_original_msg = self.br.cv2_to_imgmsg(frame_original, encoding=msg_image.encoding)
      frame_original_msg.header = msg_image.header

      frame_original_masked_msg = self.br.cv2_to_imgmsg(frame_masked, encoding=msg_image.encoding)
      frame_original_masked_msg.header = msg_image.header

      frame_grey_msg = self.br.cv2_to_imgmsg(frame_grey, encoding="mono8")
      frame_grey_msg.header = msg_image.header

      self.pub_original_frame.publish(frame_original_msg)
      self.pub_masked_frame.publish(frame_original_masked_msg)
      self.pub_grey_frame.publish(frame_grey_msg)

  def config_list(self) -> List[str]:
    return ['frame_provider_resize_frame', 'frame_provider_resize_dimension_h', 'frame_provider_resize_dimension_w', 
      'frame_provider_blur', 'frame_provider_blur_radius', 'frame_provider_cuda_enable', 'mask_type', 'mask_pct', 'mask_overlay_image_file_name',
      'controller_type']

  def validate_config(self) -> bool:
    valid = True
    
    if self.app_configuration['frame_provider_resize_frame']:
      if self.app_configuration['frame_provider_resize_dimension_h'] is None and self.app_configuration['frame_provider_resize_dimension_w'] is None:
        self.get_logger().error('Both frame_provider_resize_dimension_h and frame_provider_resize_dimension_w config entries are null')
        valid = False
      
    if self.app_configuration['frame_provider_blur']:
      if self.app_configuration['frame_provider_blur_radius'] == None:
        self.get_logger().error('The frame_provider_blur_radius config entry is null')
        valid = False

    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.counter = 0
      self.br = CvBridge()
      self.mask_svc = MaskClientAsync()

    self.frame_processor = FrameProcessor.Select(self.app_configuration, 'frame_provider_cuda_enable')

    self.image_masks = ['overlay', 'overlay_inverse']
    if any(self.app_configuration['mask_type'] in s for s in self.image_masks):      
      response = self.mask_svc.send_request(self.app_configuration['mask_overlay_image_file_name'])
      self.app_configuration['mask_overlay_image'] = self.br.imgmsg_to_cv2(response.mask)

    self.mask = Mask.Select(self.app_configuration)


def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = FrameProviderNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()