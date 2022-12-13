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
import cv2
from typing import List
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from simple_tracker_interfaces.msg import KeyPoint, KeyPointArray, BoundingBox, BoundingBoxArray
from simple_tracker_shared.control_loop_node import ControlLoopNode
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile

class BGSDetectorNode(ControlLoopNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_bgs_detector')

    # setup services, publishers and subscribers
    self.sub_masked_background_frame = self.create_subscription(Image, 'sky360/frames/masked_background/v1', 
      self.masked_background_frame_callback, 10)#, subscriber_qos_profile)
    self.pub_key_points = self.create_publisher(KeyPointArray, 'sky360/detector/bgs/key_points/v1', 10)#, publisher_qos_profile)
    self.pub_bounding_boxes = self.create_publisher(BoundingBoxArray, 'sky360/detector/bgs/bounding_boxes/v1', 10)#, publisher_qos_profile)   

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def masked_background_frame_callback(self, msg_frame:Image):
    self.msg_frame = msg_frame

  def control_loop(self):

    if self.msg_frame != None:

      frame_foreground_mask = self.br.imgmsg_to_cv2(self.msg_frame)

      key_points = self.perform_blob_detection(frame_foreground_mask, self.app_configuration['tracker_detection_sensitivity'])

      kp_array_msg = KeyPointArray()
      kp_array_msg.kps = [self._kp_to_msg(x) for x in key_points]
      self.pub_key_points.publish(kp_array_msg)

      bbox_array_msg = BoundingBoxArray()
      bbox_array_msg.boxes = [self._kp_to_bbox_msg(x) for x in key_points]
      self.pub_bounding_boxes.publish(bbox_array_msg)

  def _kp_to_msg(self, kp):

    (x, y) = kp.pt

    kp_msg = KeyPoint()
    kp_msg.x = x
    kp_msg.y = y
    kp_msg.size = kp.size

    return kp_msg

  def _kp_to_bbox_msg(self, kp):

    (x, y) = kp.pt
    size = kp.size    
    scale = 6

    x1, y1, w1, h1 = (int(x - scale * size / 2), int(y - scale * size / 2), int(scale * size), int(scale * size))

    bbox_msg = BoundingBox()
    bbox_msg.x = x1
    bbox_msg.y = y1
    bbox_msg.w = w1
    bbox_msg.h = h1

    return bbox_msg

  def perform_blob_detection(self, frame, sensitivity):
    params = cv2.SimpleBlobDetector_Params()
    # print(f"original sbd params:{params}")

    params.minRepeatability = 2
    # 5% of the width of the image
    params.minDistBetweenBlobs = int(frame.shape[1] * 0.05)
    params.minThreshold = 3
    params.filterByArea = 1
    params.filterByColor = 0
    # params.blobColor=255

    if sensitivity == 1:  # Detects small, medium and large objects
        params.minArea = 3
    elif sensitivity == 2:  # Detects medium and large objects
        params.minArea = 5
    elif sensitivity == 3:  # Detects large objects
        params.minArea = 25
    else:
        raise Exception(
            f"Unknown sensitivity option ({sensitivity}). 1, 2 and 3 is supported not {sensitivity}.")

    detector = cv2.SimpleBlobDetector_create(params)
    # params.write('params.json')
    # print("created detector")
    # blobframe=cv2.convertScaleAbs(frame)
    # print("blobframe")
    keypoints = detector.detect(frame)
    # print("ran detect")
    return keypoints

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
      self.msg_frame: Image = None
      self.br = CvBridge() 


def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = BGSDetectorNode(subscriber_qos_profile, publisher_qos_profile)

  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
      pass
  finally:
      rclpy.try_shutdown()
      node.destroy_node()

if __name__ == '__main__':
  main()