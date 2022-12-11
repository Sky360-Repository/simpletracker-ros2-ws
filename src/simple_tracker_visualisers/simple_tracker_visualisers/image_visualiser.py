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
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from typing import List
from sensor_msgs.msg import Image
from simple_tracker_interfaces.msg import Frame, TrackingState, TrackArray, Track, BoundingBox
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.utils import frame_resize
from simple_tracker_shared.qos_profiles import get_topic_subscriber_qos_profile
from .key_handler import KeyHandler

from cv_bridge import CvBridge
import cv2
 
class ImageVisualiserNode(ConfiguredNode):

  PROVISIONARY_TARGET = 1
  ACTIVE_TARGET = 2
  LOST_TARGET = 3

  def __init__(self, subscriber_qos_profile: QoSProfile):
    super().__init__('image_visualiser_node')

    self.camera_original_sub = self.create_subscription(Image, 'sky360/visualiser/original_camera_frame', self.camera_original_callback, 10)#, subscriber_qos_profile)
    self.fp_original_sub = self.create_subscription(Frame, 'sky360/visualiser/original_frame', self.fp_original_callback, 10)#, subscriber_qos_profile)
    self.fp_original_masked_sub = self.create_subscription(Frame, 'sky360/visualiser/masked_frame', self.fp_original_masked_callback, 10)#, subscriber_qos_profile)
    self.fp_grey_sub = self.create_subscription(Frame, 'sky360/visualiser/grey_frame', self.fp_grey_callback, 10)#, subscriber_qos_profile)
    self.dof_sub = self.create_subscription(Frame, 'sky360/visualiser/dense_optical_flow_frame', self.dof_callback, 10)#, subscriber_qos_profile)
    self.forground_sub = self.create_subscription(Frame, 'sky360/visualiser/foreground_mask_frame', self.foreground_callback, 10)#, subscriber_qos_profile) #sky360/frames/foreground_mask/v1
    self.masked_background_sub = self.create_subscription(Frame, 'sky360/visualiser/masked_background_frame', self.masked_background_callback, 10)#, subscriber_qos_profile)
    self.fp_annotated_sub = self.create_subscription(Frame, 'sky360/visualiser/annotated_frame', self.fp_annotated_callback, 10)#, subscriber_qos_profile)
    #self.tracking_state_sub = self.create_subscription(TrackingState, 'sky360/tracker/tracking_state/v1', self.tracking_state_callback, subscriber_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_original_callback(self, data:Image):
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
    self.key_handler.handle_key_press(cv2.waitKeyEx(1))

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
      self.key_handler = KeyHandler(self, self.br)

    self.font_size = self.app_configuration['visualiser_font_size']
    self.font_thickness = self.app_configuration['visualiser_font_thickness']
    self.bbox_line_thickness = self.app_configuration['visualiser_bbox_line_thickness']

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile(QoSReliabilityPolicy.BEST_EFFORT)

  node = ImageVisualiserNode(subscriber_qos_profile)

  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
      pass
  finally:
      rclpy.try_shutdown()
      node.destroy_node()
      #rclpy.rosshutdown()
  
if __name__ == '__main__':
  main()