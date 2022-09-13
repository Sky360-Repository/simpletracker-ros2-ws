import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from .config_entry_convertor import ConfigEntryConvertor
from .configurations_client_async import ConfigurationsClientAsync

class CameraNode(Node):

  def __init__(self):

    super().__init__('camera_node')  

    self.configuration_list = ['controller_type', 'camera_mode', 'camera_uri', 'camera_resize_dimension_h', 'camera_resize_dimension_w', 'camera_resize_frame']
    self.app_configuration = {}
    self.configuration_loaded = False

    # setup services, publishers and subscribers
    self.configuration_svc = ConfigurationsClientAsync()
    self.pub_frame = self.create_publisher(Image, 'sky360/camera/frame/original/v1', 10)    
    self.sub_config_updated = self.create_subscription(ConfigEntryUpdatedArray, 'sky360/config/updated/v1', self.config_updated_callback, 10)

    # setup timer and other helpers
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.capture_timer_callback)
    self.br = CvBridge()
    self.get_logger().info(f'Camera node is up and running.')
   
  def capture_timer_callback(self):

    if not self.configuration_loaded:
      self._load_and_validate_config()
      self.configuration_loaded = True
      self.capture = cv2.VideoCapture(self.app_configuration['camera_uri'])

    success, frame = self.capture.read()
    if success == True:

      if self.app_configuration['camera_resize_frame']:
        frame = cv2.resize(frame, (self.app_configuration['camera_resize_dimension_w'],self.app_configuration['camera_resize_dimension_h']), interpolation=cv2.INTER_AREA)

      self.pub_frame.publish(self.br.cv2_to_imgmsg(frame))

  def config_updated_callback(self, msg):

    for key in msg.keys:
      if key in self.app_configuration.keys():
        self.configuration_loaded = False
        self.get_logger().info('Receiving updated configuration notification, reload')
        break

  def _frame_resize(frame, width=None, height=None, inter=cv2.INTER_AREA):
    # initialize the dimensions of the frame to be resized and
    # grab the frame size
    dim = None
    (h, w) = frame.shape[:2]

    # if both the width and height are None, then return the
    # original frame
    if width is None and height is None:
        return frame

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the frame
    resized = cv2.resize(frame, dim, interpolation = inter)

    # return the resized frame
    return resized
  
  def _load_and_validate_config(self):

      if not self.configuration_loaded:
        self._load_config()
        
        # TODO: What is the best way of exiting out of a launch script when the configuration validation fails
        valid = self._validate_config()
        if valid == False:
          self.get_logger().error('Camera configuration is invalid')

        self.configuration_loaded = True
        self.capture = cv2.VideoCapture(self.app_configuration['camera_uri'])

  def _load_config(self):

    self.get_logger().info(f'Loading configuration list.')

    response = self.configuration_svc.send_request(self.configuration_list)
    for config_item in response.entries:
      self.app_configuration[config_item.key] = ConfigEntryConvertor.Convert(config_item.type, config_item.value)

  def _validate_config(self):
    self.get_logger().info(f'Validating configuration.')

    valid = True

    if self.app_configuration['controller_type'] != 'camera':
      self.get_logger().error('Only the camera controller is supported')
      valid = False

    if self.app_configuration['camera_uri'] == None:
      self.get_logger().error('The camera_uri config entry is null')
      valid = False

    if self.app_configuration['camera_resize_frame']:
      if self.app_configuration['camera_resize_dimension_h'] == None:
        self.get_logger().error('The camera_resize_dimension_h config entry is null')
        valid = False
      
      if self.app_configuration['camera_resize_dimension_w'] == None:
        self.get_logger().error('The camera_resize_dimension_w config entry is null')
        valid = False

    return valid


def main(args=None):

  rclpy.init(args=args)
  image_publisher = CameraNode()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()