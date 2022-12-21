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
#from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray
#from geometry_msgs.msg import Pose2D
from simple_tracker_shared.control_loop_node import ConfiguredNode
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile

class BGSDetectorNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_bgs_detector')

    # setup services, publishers and subscribers
    self.sub_masked_background_frame = self.create_subscription(Image, 'sky360/frames/masked_background/v1', 
      self.masked_background_frame_callback, 10)#, subscriber_qos_profile)
    self.pub_bounding_boxes = self.create_publisher(BoundingBox2DArray, 'sky360/detector/bgs/bounding_boxes/v1', 10)#, publisher_qos_profile)   

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def masked_background_frame_callback(self, msg_frame:Image):

    if msg_frame != None:

      frame_foreground_mask = self.br.imgmsg_to_cv2(msg_frame)

      key_points = self.perform_blob_detection(frame_foreground_mask, self.app_configuration['tracker_detection_sensitivity'])

      bbox_array_msg = BoundingBox2DArray()
      bbox_array_msg.header = msg_frame.header
      [bbox_array_msg.boxes.append(self._kp_to_bbox_msg(x)) for x in key_points]
      self.pub_bounding_boxes.publish(bbox_array_msg)

  def _kp_to_bbox_msg(self, kp):

    (x, y) = kp.pt
    size = kp.size    
    scale = 6

    x1, y1, w1, h1 = ((x - scale * size / 2), (y - scale * size / 2), (scale * size), (scale * size))

    bbox_msg = BoundingBox2D()
    ## bbox_msg.source_img = deepcopy(req.image)
    bbox_msg.center.position.x = float(x1+(w1/2))
    bbox_msg.center.position.y = float(y1+(h1/2))
    bbox_msg.center.theta = 0.0

    bbox_msg.size_x = float(w1)
    bbox_msg.size_y = float(h1)

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