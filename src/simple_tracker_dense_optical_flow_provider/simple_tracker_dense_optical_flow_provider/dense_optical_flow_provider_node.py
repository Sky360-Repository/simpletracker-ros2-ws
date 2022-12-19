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
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame
from simple_tracker_shared.control_loop_node import ControlLoopNode
from simple_tracker_shared.frame_processor import FrameProcessor
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .dense_optical_flow import DenseOpticalFlow

class DenseOpticalFlowProviderNode(ControlLoopNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_dense_optical_flow_provider')

    # setup services, publishers and subscribers
    self.sub_grey_frame = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.grey_frame_callback, 10)#, subscriber_qos_profile)
    self.pub_dense_optical_flow_frame = self.create_publisher(Frame, 'sky360/frames/dense_optical_flow/v1', 10)#, publisher_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def grey_frame_callback(self, msg_frame:Frame):
    self.msg_frame = msg_frame

  def control_loop(self):

    if self.msg_frame != None:

      self.frame_grey = self.br.imgmsg_to_cv2(self.msg_frame.frame)

      optical_flow_frame = self.frame_processor.process_optical_flow(self.dense_optical_flow, self.frame_grey, None)
      
      frame_optical_flow_msg = Frame()
      frame_optical_flow_msg.epoch = self.msg_frame.epoch
      frame_optical_flow_msg.fps = self.msg_frame.fps
      frame_optical_flow_msg.frame_count = self.msg_frame.frame_count
      frame_optical_flow_msg.frame = self.br.cv2_to_imgmsg(optical_flow_frame)
      self.pub_dense_optical_flow_frame.publish(frame_optical_flow_msg)


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
      self.msg_frame:Frame = None

    self.frame_processor = FrameProcessor.Select(self.app_configuration, 'dense_optical_cuda_enable')
    self.dense_optical_flow = DenseOpticalFlow.Select(self.app_configuration)
    

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = DenseOpticalFlowProviderNode(subscriber_qos_profile, publisher_qos_profile)

  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
      pass
  finally:
      rclpy.try_shutdown()
      node.destroy_node()

if __name__ == '__main__':
  main()