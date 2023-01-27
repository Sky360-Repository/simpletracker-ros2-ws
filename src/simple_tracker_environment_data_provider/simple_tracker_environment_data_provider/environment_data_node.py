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
from simple_tracker_shared.control_loop_node import ConfiguredNode
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .cloud_estimator import CloudEstimator

class EnvironmentDataNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_cloud_estimation_provider')

    # setup services, publishers and subscribers    
    self.sub_camera = self.create_subscription(Image, 'sky360/frames/original', self.camera_callback, 10)#, subscriber_qos_profile)

    self.timer = self.create_timer(self.cloud_sampler_timer_period(), self.cloud_sampler)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_callback(self, msg_image:Image):
    self.msg_image = msg_image

  def cloud_sampler(self):
    
    if self.msg_image != None:
        estimation = self.datetime_cloud_estimator.estimate(self.br.imgmsg_to_cv2(self.msg_image))
        self.get_logger().info(f'{self.get_name()} Day time cloud estimation --> {estimation}')
        estimation = self.nighttime_cloud_estimator.estimate(self.br.imgmsg_to_cv2(self.msg_image))
        self.get_logger().info(f'{self.get_name()} Night time cloud estimation --> {estimation}')        

  def cloud_sampler_timer_period(self) -> int:
    return 300

  def config_list(self) -> List[str]:
    return []

  def validate_config(self) -> bool:
    valid = True
    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.counter = 0
      self.br = CvBridge()

    self.msg_image = None
    self.datetime_cloud_estimator = CloudEstimator.Day(self.app_configuration)
    self.nighttime_cloud_estimator = CloudEstimator.Night(self.app_configuration)

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = EnvironmentDataNode(subscriber_qos_profile, publisher_qos_profile)

  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
      pass
  finally:
      rclpy.try_shutdown()
      node.destroy_node()


if __name__ == '__main__':
  main()