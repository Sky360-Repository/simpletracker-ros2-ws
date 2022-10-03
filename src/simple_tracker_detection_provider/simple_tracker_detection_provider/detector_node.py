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

class DetectorNode(ConfiguredNode):

  def __init__(self):
    super().__init__('sky360_detector')

    # setup services, publishers and subscribers
    self.sub_masked_background_frame = self.create_subscription(Frame, 'sky360/frames/masked_background/v1', self.masked_background_frame_callback, 10)
    self.pub_key_points = self.create_publisher(KeyPointArray, 'sky360/detector/key_points/v1', 10)
    self.pub_bounding_boxes = self.create_publisher(BoundingBoxArray, 'sky360/detector/bounding_boxes/v1', 10)   

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def masked_background_frame_callback(self, data:Frame):

    # TODO: This configuration update thing needs to happen in the background
    if not self.configuration_loaded:
      self._load_and_validate_config()
      self.configuration_loaded = True

    frame_foreground_mask = self.br.imgmsg_to_cv2(data.frame)

    key_points = perform_blob_detection(frame_foreground_mask, self.app_configuration['tracker_detection_sensitivity'])

    kp_array_msg = KeyPointArray()
    kp_array_msg.epoch = data.epoch
    kp_array_msg.frame_count = data.frame_count
    kp_array_msg.kps = [self._kp_to_msg(x) for x in key_points]
    self.pub_key_points.publish(kp_array_msg)

    bbox_array_msg = BoundingBoxArray()
    bbox_array_msg.epoch = data.epoch    
    bbox_array_msg.frame_count = data.frame_count
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
  detector_node = DetectorNode()
  rclpy.spin(detector_node)
  detector_node.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()