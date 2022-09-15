import datetime
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
#from simple_tracker_interfaces.msg import KeyPoint
#from simple_tracker_interfaces.msg import KeyPointArray
from simple_tracker_interfaces.msg import BoundingBox
from simple_tracker_interfaces.msg import BoundingBoxArray
from .config_entry_convertor import ConfigEntryConvertor
from .configurations_client_async import ConfigurationsClientAsync
from.utils import perform_blob_detection, kp_to_bbox, get_sized_bbox

class DetectorNode(Node):

  def __init__(self):

    super().__init__('sky360_detector')  

    self.configuration_list = ['tracker_detection_sensitivity', 'bbox_size']
    self.app_configuration = {}
    self.configuration_loaded = False

    # setup services, publishers and subscribers
    self.configuration_svc = ConfigurationsClientAsync()
    self.sub_masked_background_frame = self.create_subscription(Image, 'sky360/frames/masked_background/v1', self.masked_background_frame_callback, 10)
    #self.pub_key_points = self.create_publisher(KeyPointArray, 'sky360/detection/key_points/v1', 10)
    #self.pub_key_point = self.create_publisher(KeyPoint, 'sky360/detection/key_point/v1', 10)
    self.pub_bbox = self.create_publisher(BoundingBox, 'sky360/detection/bounding_box/v1', 10)
    #self.pub_bboxes = self.create_publisher(BoundingBoxArray, 'sky360/detection/bounding_boxes/v1', 10)    
    self.pub_sized_bbox = self.create_publisher(BoundingBox, 'sky360/detection/bounding_box/sized/v1', 10)
    #self.pub_sized_bboxes = self.create_publisher(BoundingBox, 'sky360/detection/bounding_boxes/sized/v1', 10)
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

      #self.get_logger().info('Found keypoints --> {key_points}')
      #self.get_logger().info('Bboxes --> {bboxes}')

      #bboxes = [self._kp_to_bbox(x) for x in key_points]
      #boxes = [self._bbox_to_msg(x) for x in bboxes]

      #bbox_array_msg = BoundingBoxArray()
      #bbox_array_msg.boxes = boxes
      #self.pub_bboxes.publish(bbox_array_msg)

      [self._publish_keypoint(x) for x in key_points]

  def _publish_keypoint(self, kp):

    (x, y) = kp.pt
    kp_size = kp.size    
    kp_scale = 6

    #kp_msg = KeyPoint()
    #kp_msg.x = x
    #kp_msg.y = y
    #kp_msg.size = kp_size

    x1, y1, w1, h1 = (int(x - kp_scale * kp_size / 2), int(y - kp_scale * kp_size / 2), int(kp_scale * kp_size), int(kp_scale * kp_size))

    bbox_msg = BoundingBox()
    bbox_msg.x = x1
    bbox_msg.y = y1
    bbox_msg.w = w1
    bbox_msg.h = h1

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

    #self.pub_key_point.publish(kp_msg)
    self.pub_bbox.publish(bbox_msg)
    self.pub_sized_bbox.publish(sized_bbox_msg)

  def config_updated_callback(self, msg):

    for key in msg.keys:
      if key in self.app_configuration.keys():
        self.configuration_loaded = False
        self.get_logger().info('Receiving updated configuration notification, reload')
        break

  def _bbox_to_msg(self, bbox):
    x, y, w, h = bbox
    bbox_msg = BoundingBox()
    bbox_msg.x = x
    bbox_msg.y = y
    bbox_msg.w = w
    bbox_msg.h = h

    return bbox_msg

  def _kp_to_bbox(self, kp):

    (x, y) = kp.pt
    size = kp.size    
    scale = 6

    kp_msg = KeyPoint()
    kp_msg.x = x
    kp_msg.y = y
    kp_msg.size = size

    #print(f'kp_to_bbox x, y:{(x, y)}, size:{size}, scale:{scale}, new size:{scale * size}')
    return (int(x - scale * size / 2), int(y - scale * size / 2), int(scale * size), int(scale * size))

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