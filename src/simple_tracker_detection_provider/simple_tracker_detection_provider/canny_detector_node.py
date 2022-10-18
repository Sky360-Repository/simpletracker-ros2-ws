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
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame
from simple_tracker_interfaces.msg import KeyPoint
from simple_tracker_interfaces.msg import KeyPointArray
from simple_tracker_interfaces.msg import BoundingBox
from simple_tracker_interfaces.msg import BoundingBoxArray
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.utils import perform_blob_detection

class CannyDetectorNode(ConfiguredNode):

  def __init__(self):
    super().__init__('sky360_canny_detector')

    # setup services, publishers and subscribers
    self.sub_grey_frame = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.grey_frame_callback, 10)    
    self.pub_bounding_boxes = self.create_publisher(BoundingBoxArray, 'sky360/detector/canny/bounding_boxes/v1', 10)   

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def grey_frame_callback(self, data:Frame):

    frame_grey = self.br.imgmsg_to_cv2(data.frame)

    # TODO:
   



  def config_list(self) -> List[str]:
    return ['tracker_detection_sensitivity']

  def validate_config(self) -> bool:
    valid = True

    if self.app_configuration['tracker_detection_sensitivity'] == None:
      self.get_logger().error('The tracker_detection_sensitivity config entry is null')
      valid = False
      
    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge() 


def main(args=None):

  rclpy.init(args=args)
  detector_node = CannyDetectorNode()
  rclpy.spin(detector_node)
  detector_node.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()