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

import os
import rclpy
from typing import List
from simple_tracker_interfaces.msg import Frame, CameraFrame, TrackingState, TrackArray, Track, BoundingBox
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.utils import frame_resize
from .mask_client_async import MaskClientAsync

from cv_bridge import CvBridge
import cv2
 
class ImageVisualiserNode(ConfiguredNode):

  PROVISIONARY_TARGET = 1
  ACTIVE_TARGET = 2
  LOST_TARGET = 3

  def __init__(self):
    super().__init__('image_visualiser_node')

    self.camera_original_sub = self.create_subscription(CameraFrame, 'sky360/visualiser/original_camera_frame', self.camera_original_callback, 10)
    self.fp_original_sub = self.create_subscription(Frame, 'sky360/visualiser/original_frame', self.fp_original_callback, 10)
    self.fp_original_masked_sub = self.create_subscription(Frame, 'sky360/visualiser/masked_frame', self.fp_original_masked_callback, 10)
    self.fp_grey_sub = self.create_subscription(Frame, 'sky360/visualiser/grey_frame', self.fp_grey_callback, 10)
    self.dof_sub = self.create_subscription(Frame, 'sky360/visualiser/dense_optical_flow_frame', self.dof_callback, 10)
    self.forground_sub = self.create_subscription(Frame, 'sky360/visualiser/foreground_mask_frame', self.foreground_callback, 10) #sky360/frames/foreground_mask/v1
    self.masked_background_sub = self.create_subscription(Frame, 'sky360/visualiser/masked_background_frame', self.masked_background_callback, 10)
    self.fp_annotated_sub = self.create_subscription(Frame, 'sky360/visualiser/annotated_frame', self.fp_annotated_callback, 10)
    #self.tracking_state_sub = self.create_subscription(TrackingState, 'sky360/tracker/tracking_state/v1', self.tracking_state_callback, 10)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_original_callback(self, data:CameraFrame):
    camera_original_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("camera/original", camera_original_frame)
    cv2.waitKey(1)

  def fp_original_callback(self, data:Frame):
    original_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("fp/original", original_frame)
    cv2.waitKey(1)

  def fp_original_masked_callback(self, data:Frame):
    masked_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("fp/masked", masked_frame)
    cv2.waitKey(1)

  def fp_grey_callback(self, data:Frame):
    camera_grey_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("fp/grey", camera_grey_frame)
    cv2.waitKey(1)

  def dof_callback(self, data:Frame):
    dof_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("dense-optical-flow", dof_frame)
    cv2.waitKey(1)

  def foreground_callback(self, data:Frame):
    foreground_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("foreground-mask", foreground_frame)
    cv2.waitKey(1)

  def masked_background_callback(self, data:Frame):
    masked_background_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("masked-background", masked_background_frame)
    cv2.waitKey(1)

  def tracking_state_callback(self, data:TrackingState):
    if self.configuration_loaded == False:
      pass

    msg = f"(Sky360) Tracker Status: trackable:{data.trackable}, alive:{data.alive}, started:{data.started}, ended:{data.ended}, frame count:{data.frame_count}, frame epoch:{data.epoch}, fps:{data.fps} "
    if self.app_configuration['visualiser_log_status_to_console']:
      self.get_logger().info(msg)

  def fp_annotated_callback(self, data:Frame):
    annotated_frame = self.br.imgmsg_to_cv2(data.frame)
    annotated_frame = self._resize(annotated_frame)
    cv2.imshow("fp/annotated", annotated_frame)
    k = cv2.waitKey(1)
    if k > 0:
      self.get_logger().info(f'Key press {k} captured.')
      if k == 32:
        # spacebar
        self.update_mask_test()

  def _resize(self, frame):
    if self.app_configuration['visualiser_resize_frame']:
      frame = frame_resize(frame, height=self.app_configuration['visualiser_resize_dimension_h'], width=self.app_configuration['visualiser_resize_dimension_w'])
    return frame


  def config_list(self) -> List[str]:
    return ['visualiser_font_size', 'visualiser_font_thickness', 'visualiser_bbox_line_thickness', 'visualiser_bbox_size', 
      'visualiser_log_status_to_console', 'visualiser_resize_frame', 'visualiser_resize_dimension_h', 'visualiser_resize_dimension_w']

  def validate_config(self) -> bool:
    valid = True

    if self.app_configuration['visualiser_font_size'] == None:
      self.get_logger().error('The visualiser_font_size config entry is null')
      valid = False

    if self.app_configuration['visualiser_font_thickness'] == None:
      self.get_logger().error('The visualiser_font_thickness config entry is null')
      valid = False

    return valid  

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()
      self.font_colour = (50, 170, 50)
      self.mask_svc = MaskClientAsync()

    self.font_size = self.app_configuration['visualiser_font_size']
    self.font_thickness = self.app_configuration['visualiser_font_thickness']
    self.bbox_line_thickness = self.app_configuration['visualiser_bbox_line_thickness']

  def update_mask_test(self):

    mask_file_path = '/workspaces/simpletracker-ros2-ws/beeks_mask.jpg'

    if os.path.exists(mask_file_path) == False:
      self.get_logger().error(f'Mask path {mask_file_path} does not exist.')

    mask_image = cv2.imread(mask_file_path, cv2.IMREAD_GRAYSCALE)
    msg_mask_image = self.br.cv2_to_imgmsg(mask_image)    
    mask_type = 'overlay_inverse'
    file_name = 'beeks_mask.jpg'

    response = self.mask_svc.send_request(msg_mask_image, mask_type,file_name)
    self.get_logger().info(f'Mask update response: {response}')

def main(args=None):

  rclpy.init(args=args)
  image_visualiser = ImageVisualiserNode()
  rclpy.spin(image_visualiser)
  image_visualiser.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()