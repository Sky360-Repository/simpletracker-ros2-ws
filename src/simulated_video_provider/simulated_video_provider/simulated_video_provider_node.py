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
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
import cv2
from cv_bridge import CvBridge
import os
from rclpy.executors import ExternalShutdownException
from typing import List
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from simple_tracker_shared.control_loop_node import ConfiguredNode
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .synthetic_data_generator import SyntheticDataGenerator
from simple_tracker_shared.utils import frame_resize

class SimulatedVideoProviderNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_simulated_video_provider')

    # setup services, publishers and subscribers
    self.pub_synthetic_frame = self.create_publisher(Image, 'sky360/simulation/v1', 10)#, publisher_qos_profile)

    self.timer = self.create_timer(self.timer_period, self.timer_callback)  

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def timer_callback(self):

    if not self.still_frame_image is None:

      self.counter += 1
      frame_synthetic = self.still_frame_image
      if self.counter > 150:
        frame_synthetic = self.create_and_add_synthetic_data(self.still_frame_image)      

      time_msg = self.get_time_msg()

      frame_synthetic_msg = self.br.cv2_to_imgmsg(frame_synthetic, encoding="bgr8")
      frame_synthetic_msg.header.stamp = time_msg
      frame_synthetic_msg.header.frame_id = 'synthetic'

      self.pub_synthetic_frame.publish(frame_synthetic_msg)

  def create_and_add_synthetic_data(self, frame_original):

    self.shape = frame_original.shape[:2]
    self.height = self.shape[0]
    self.width = self.shape[1]

    frame_with_data = self.data_generator.generate_and_add(frame_original)

    return frame_with_data

  def config_list(self) -> List[str]:
    return ['frame_provider_resize_frame', 'frame_provider_resize_dimension_h', 'frame_provider_resize_dimension_w']

  def validate_config(self) -> bool:
    valid = True
    return valid          

  def on_config_loaded(self, init: bool):
    if init:
      self.timer_period = 0.05
      self.br = CvBridge()
      self.still_frame_image = None
      self.counter = 0

    file_name = 'frame_1.jpg'
    frames_folder = self.videos_folder = os.path.join(get_package_share_directory('simulated_video_provider'), 'still_frames')
    frame_file_path = os.path.join(frames_folder, file_name)

    if os.path.exists(frame_file_path) == False:
      self.get_logger().error(f'Still frame path {frame_file_path} does not exist.')

    self.still_frame_image = cv2.imread(frame_file_path, cv2.IMREAD_COLOR)

    if self.app_configuration['frame_provider_resize_frame']: 
      self.still_frame_image = frame_resize(self.still_frame_image, 
        width=self.app_configuration['frame_provider_resize_dimension_w'], 
        height=self.app_configuration['frame_provider_resize_dimension_h'])

    #self.data_generator = SyntheticDataGenerator.Plane(self.app_configuration, self.get_logger())
    self.data_generator = SyntheticDataGenerator.Drone(self.app_configuration, self.get_logger())

  def get_time_msg(self):
    time_msg = Time()
    msg_time = self.get_clock().now().seconds_nanoseconds()

    time_msg.sec = int(msg_time[0])
    time_msg.nanosec = int(msg_time[1])
    return time_msg

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = SimulatedVideoProviderNode(subscriber_qos_profile, publisher_qos_profile)

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
