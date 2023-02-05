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
from typing import List
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_interfaces.msg import ObserverDayNight
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .day_night_classifier import DayNightEstimator, DayNightEnum

class DayNightClassifierNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_day_night_estimator')

    self.pub_environment_data = self.create_publisher(ObserverDayNight, 'sky360/observer/day_night_classifier', 10)#, publisher_qos_profile)

    # setup services, publishers and subscribers    
    self.sub_camera = self.create_subscription(Image, 'sky360/frames/original', self.camera_callback, 10)#, subscriber_qos_profile)

    self.timer = self.create_timer(self.day_night_sampler_timer_period(), self.day_night_classifier)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_callback(self, msg_image:Image):
    self.msg_image = msg_image

  def day_night_classifier(self):
    
    if self.msg_image != None:

      result, average_brightness = self.day_night_estimator.estimate(self.br.imgmsg_to_cv2(self.msg_image))
      self.get_logger().info(f'{self.get_name()} Day/Night classifier --> {result}, {average_brightness}')

      day_night_msg = ObserverDayNight()
      day_night_msg.is_night = result == DayNightEnum.Night
      day_night_msg.avg_brightness = average_brightness
      self.pub_environment_data.publish(day_night_msg)

  def day_night_sampler_timer_period(self) -> int:
    return 30

  def config_list(self) -> List[str]:
    return ['observer_day_night_brightness_threshold']

  def validate_config(self) -> bool:
    valid = True
    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.counter = 0
      self.br = CvBridge()

    self.msg_image = None
    self.day_night_estimator = DayNightEstimator.Classifier(self.app_configuration)

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = DayNightClassifierNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()