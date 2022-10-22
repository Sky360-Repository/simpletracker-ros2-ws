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
import cv2
from typing import List
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import CameraFrame
from simple_tracker_interfaces.msg import Frame
from simple_tracker_shared.control_loop_node import ControlLoopNode
from simple_tracker_shared.frame_processor import FrameProcessor
from simple_tracker_shared.utils import frame_resize
from .mask import Mask
from .mask_client_async import MaskClientAsync

class FrameProviderNode(ControlLoopNode):

  def __init__(self):
    super().__init__('sky360_frame_provider')

    # setup services, publishers and subscribers    
    self.sub_camera = self.create_subscription(CameraFrame, 'sky360/camera/original/v1', self.camera_callback, 10)
    self.pub_original_frame = self.create_publisher(Frame, 'sky360/frames/original/v1', 10)
    self.pub_masked_frame = self.create_publisher(Frame, 'sky360/frames/masked/v1', 10)
    self.pub_grey_frame = self.create_publisher(Frame, 'sky360/frames/grey/v1', 10)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_callback(self, msg_image:Image):
    self.msg_image = msg_image

  def control_loop(self):

    if self.msg_image != None:

      self.counter += 1

      frame_original, frame_grey, frame_masked = self.frame_processor.process_for_frame_provider(self.mask, 
        self.br.imgmsg_to_cv2(self.msg_image.frame), stream=None)

      frame_original_msg = Frame()
      frame_original_msg.epoch = self.msg_image.epoch
      frame_original_msg.fps = self.msg_image.fps
      frame_original_msg.frame_count = self.counter
      frame_original_msg.frame = self.br.cv2_to_imgmsg(frame_original)

      self.pub_original_frame.publish(frame_original_msg)

      frame_original_masked_msg = Frame()
      frame_original_masked_msg.epoch = self.msg_image.epoch
      frame_original_masked_msg.fps = self.msg_image.fps
      frame_original_masked_msg.frame_count = self.counter
      frame_original_masked_msg.frame = self.br.cv2_to_imgmsg(frame_masked)

      self.pub_masked_frame.publish(frame_original_masked_msg)

      frame_grey_msg = Frame()
      frame_grey_msg.epoch = self.msg_image.epoch
      frame_grey_msg.fps = self.msg_image.fps
      frame_grey_msg.frame_count = self.counter
      frame_grey_msg.frame = self.br.cv2_to_imgmsg(frame_grey)

      self.pub_grey_frame.publish(frame_grey_msg)

  def config_list(self) -> List[str]:
    return ['frame_provider_resize_frame', 'frame_provider_resize_dimension_h', 'frame_provider_resize_dimension_w', 
      'frame_provider_blur', 'frame_provider_blur_radius', 'frame_provider_cuda_enable', 'mask_type', 'mask_pct', 'mask_overlay_image_file_name']

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
      self.msg_image: Image = None
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
  frame_provider = FrameProviderNode()
  rclpy.spin(frame_provider)
  frame_provider.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()