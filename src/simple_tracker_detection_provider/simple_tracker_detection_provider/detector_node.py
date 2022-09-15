import datetime
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from simple_tracker_interfaces.msg import KeyPoint
from simple_tracker_interfaces.msg import KeyPointArray
from simple_tracker_interfaces.msg import BoundingBox
from simple_tracker_interfaces.msg import BoundingBoxArray
from .config_entry_convertor import ConfigEntryConvertor
from .configurations_client_async import ConfigurationsClientAsync
from.utils import perform_blob_detection

class DetectorNode(Node):

  def __init__(self):

    super().__init__('sky360_detector')  

    self.configuration_list = ['tracker_detection_sensitivity', 'bbox_size']
    self.app_configuration = {}
    self.configuration_loaded = False

    # setup services, publishers and subscribers
    self.configuration_svc = ConfigurationsClientAsync()
    self.sub_masked_background_frame = self.create_subscription(Image, 'sky360/frames/masked_background/v1', self.masked_background_frame_callback, 10)
    self.pub_key_points = self.create_publisher(KeyPointArray, 'sky360/detector/key_points/v1', 10)
    self.pub_bounding_boxes = self.create_publisher(BoundingBoxArray, 'sky360/detector/bounding_boxes/v1', 10)    
    self.pub_sized_bounding_boxes = self.create_publisher(BoundingBoxArray, 'sky360/detector/bounding_boxes/sized/v1', 10)
    self.sub_config_updated = self.create_subscription(ConfigEntryUpdatedArray, 'sky360/config/updated/v1', self.config_updated_callback, 10)

    # setup timer and other helpers
    self.br = CvBridge()
    
    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def masked_background_frame_callback(self, data):

    # TODO: This configuration update thing needs to happen in the background
    if not self.configuration_loaded:
      self._load_and_validate_config()
      self.configuration_loaded = True

    frame_foreground_mask = self.br.imgmsg_to_cv2(data)

    key_points = perform_blob_detection(frame_foreground_mask, self.app_configuration['tracker_detection_sensitivity'])

    if len(key_points) > 0:

      kp_array_msg = KeyPointArray()
      kp_array_msg.kps = [self._kp_to_msg(x) for x in key_points]
      self.pub_key_points.publish(kp_array_msg)

      bbox_array_msg = BoundingBoxArray()
      bbox_array_msg.boxes = [self._kp_to_bbox_msg(x) for x in key_points]
      self.pub_bounding_boxes.publish(bbox_array_msg)

      sized_bbox_array_msg = BoundingBoxArray()
      sized_bbox_array_msg.boxes = [self._kp_to_sized_bbox_msg(x) for x in key_points]
      self.pub_sized_bounding_boxes.publish(sized_bbox_array_msg)

  def config_updated_callback(self, msg):

    for key in msg.keys:
      if key in self.app_configuration.keys():
        self.configuration_loaded = False
        self.get_logger().info('Receiving updated configuration notification, reload')
        break

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

  def _kp_to_sized_bbox_msg(self, kp):

    (x, y) = kp.pt
    size = kp.size    
    scale = 6

    x1, y1, w1, h1 = (int(x - scale * size / 2), int(y - scale * size / 2), int(scale * size), int(scale * size))

    bbox_size = self.app_configuration['bbox_size']
    x2 = int(x1+(w1/2)) - int(bbox_size/2)
    y2 = int(y1+(h1/2)) - int(bbox_size/2)
    w2 = bbox_size
    h2 = bbox_size

    sized_bbox_msg = BoundingBox()
    sized_bbox_msg.x = x2
    sized_bbox_msg.y = y2
    sized_bbox_msg.w = w2
    sized_bbox_msg.h = h2

    return sized_bbox_msg

  def _load_and_validate_config(self):

      if not self.configuration_loaded:
        self._load_config()
        
        # TODO: What is the best way of exiting out of a launch script when the configuration validation fails
        valid = self._validate_config()
        if valid == False:
          self.get_logger().error('Detector Node configuration is invalid')

        self.configuration_loaded = True

  def _load_config(self):
    #self.get_logger().info(f'Loading configuration list.')

    response = self.configuration_svc.send_request(self.configuration_list)
    for config_item in response.entries:
      self.app_configuration[config_item.key] = ConfigEntryConvertor.Convert(config_item.type, config_item.value)

  def _validate_config(self):
    #self.get_logger().info(f'Validating configuration.')

    valid = True

    if self.app_configuration['tracker_detection_sensitivity'] == None:
      self.get_logger().error('The tracker_detection_sensitivity config entry is null')
      valid = False
      
    return valid

def main(args=None):

  rclpy.init(args=args)
  detector_node = DetectorNode()
  rclpy.spin(detector_node)
  detector_node.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()