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
from rclpy.qos import QoSProfile, QoSPresetProfiles, qos_profile_sensor_data
from typing import List
from sensor_msgs.msg import Image
from simple_tracker_interfaces.msg import TrackingState
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.utils import frame_resize
from simple_tracker_shared.qos_profiles import get_topic_subscriber_qos_profile
from .key_handler import KeyHandler
from cv_bridge import CvBridge
import cv2
 
class ImageVisualiserNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile):
    super().__init__('image_visualiser_node')

    self.camera_original_sub = self.create_subscription(Image, 'sky360/visualiser/original_camera_frame', self.camera_original_callback, subscriber_qos_profile)
    self.fp_original_sub = self.create_subscription(Image, 'sky360/visualiser/original_frame', self.fp_original_callback, subscriber_qos_profile)
    self.fp_original_masked_sub = self.create_subscription(Image, 'sky360/visualiser/masked_frame', self.fp_original_masked_callback, subscriber_qos_profile)
    self.fp_grey_sub = self.create_subscription(Image, 'sky360/visualiser/grey_frame', self.fp_grey_callback, subscriber_qos_profile)
    self.dof_sub = self.create_subscription(Image, 'sky360/visualiser/dense_optical_flow_frame', self.dof_callback, subscriber_qos_profile)
    self.forground_sub = self.create_subscription(Image, 'sky360/visualiser/foreground_mask_frame', self.foreground_callback, subscriber_qos_profile)
    self.masked_background_sub = self.create_subscription(Image, 'sky360/visualiser/masked_background_frame', self.masked_background_callback, subscriber_qos_profile)
    self.fp_annotated_sub = self.create_subscription(Image, 'sky360/visualiser/annotated_frame', self.fp_annotated_callback, subscriber_qos_profile)
    self.tracking_state_sub = self.create_subscription(TrackingState, 'sky360/visualiser/tracking_state', self.tracking_state_callback, subscriber_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_original_callback(self, data:Image):
    camera_original_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("camera/original", camera_original_frame)
    cv2.waitKey(1)

  def fp_original_callback(self, data:Image):
    original_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("fp/original", original_frame)
    cv2.waitKey(1)

  def fp_original_masked_callback(self, data:Image):
    masked_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("fp/masked", masked_frame)
    cv2.waitKey(1)

  def fp_grey_callback(self, data:Image):
    camera_grey_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("fp/grey", camera_grey_frame)
    cv2.waitKey(1)

  def dof_callback(self, data:Image):
    dof_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("dense-optical-flow", dof_frame)
    cv2.waitKey(1)

  def foreground_callback(self, data:Image):
    foreground_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("foreground-mask", foreground_frame)
    cv2.waitKey(1)

  def masked_background_callback(self, data:Image):
    masked_background_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("masked-background", masked_background_frame)
    cv2.waitKey(1)

  def tracking_state_callback(self, data:TrackingState):
    if self.configuration_loaded == False:
      pass

    msg = f"(Sky360) Tracker Status: trackable:{data.trackable}, alive:{data.alive}, started:{data.started}, ended:{data.ended} "
    if self.app_configuration['visualiser_log_status_to_console']:
      self.get_logger().info(msg)

  def fp_annotated_callback(self, data:Image):
    annotated_frame = self.br.imgmsg_to_cv2(data)
    annotated_frame = self._resize(annotated_frame)
    cv2.imshow("fp/annotated", annotated_frame)
    self.key_handler.handle_key_press(cv2.waitKeyEx(1))

  def _resize(self, frame):
    if self.app_configuration['visualiser_resize_frame']:
      frame = frame_resize(frame, height=self.app_configuration['visualiser_resize_dimension_h'], width=self.app_configuration['visualiser_resize_dimension_w'])
    return frame

  def config_list(self) -> List[str]:
    return ['visualiser_log_status_to_console', 'visualiser_resize_frame', 'visualiser_resize_dimension_h', 'visualiser_resize_dimension_w']

  def validate_config(self) -> bool:
    valid = True
    return valid  

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()
      self.font_colour = (50, 170, 50)
      self.key_handler = KeyHandler(self, self.br)

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = qos_profile_sensor_data #get_topic_subscriber_qos_profile()

  node = ImageVisualiserNode(subscriber_qos_profile)

  runner = NodeRunner(node)
  runner.run()
  
if __name__ == '__main__':
  main()