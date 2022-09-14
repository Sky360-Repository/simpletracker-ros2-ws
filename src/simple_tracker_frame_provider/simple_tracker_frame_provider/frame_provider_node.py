import datetime
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from .mask import Mask
from .config_entry_convertor import ConfigEntryConvertor
from .configurations_client_async import ConfigurationsClientAsync

class FrameProviderNode(Node):

  def __init__(self):

    super().__init__('frame_provider_node')  

    self.configuration_list = ['frame_provider_resize_frame', 'frame_provider_resize_dimension_h', 'frame_provider_resize_dimension_w', 
      'frame_provider_noise_reduction', 'frame_provider_blur_radius', 'tracker_cuda_enable', 'mask_type', 'mask_pct']
    self.app_configuration = {}
    self.configuration_loaded = False
    self.mask_initialised = False

    # setup services, publishers and subscribers
    self.configuration_svc = ConfigurationsClientAsync()
    self.sub_camera = self.create_subscription(Image, 'sky360/camera/original/v1', self.camera_callback, 10)
    self.pub_original_frame = self.create_publisher(Image, 'sky360/frames/original/v1', 10)
    self.pub_grey_frame = self.create_publisher(Image, 'sky360/frames/grey/v1', 10)
    self.sub_config_updated = self.create_subscription(ConfigEntryUpdatedArray, 'sky360/config/updated/v1', self.config_updated_callback, 10)

    # setup timer and other helpers
    self.br = CvBridge()
    self.get_logger().info(f'Frame Provider node is up and running.')
   
  def camera_callback(self, data):

    # TODO: This configuration update thing needs to happen in the background
    if not self.configuration_loaded:
      self._load_and_validate_config()
      self._setup_mask()
      self.configuration_loaded = True

    frame = self.br.imgmsg_to_cv2(data)

    if self.app_configuration['frame_provider_resize_frame']:      
      frame = cv2.resize(frame, (self.app_configuration['frame_provider_resize_dimension_w'],self.app_configuration['frame_provider_resize_dimension_h']), interpolation=cv2.INTER_AREA)

    # apply mask
    if not self.mask_initialised:
      self.mask.initialise(frame)
      self.mask_initialised = True

    frame = self.mask.apply(frame)

    #grey
    frame_grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # blur
    if self.app_configuration['frame_provider_noise_reduction']:
      frame_grey = cv2.GaussianBlur(frame_grey, (self.app_configuration['frame_provider_blur_radius'], self.app_configuration['frame_provider_blur_radius']), 0)

    self.pub_original_frame.publish(self.br.cv2_to_imgmsg(frame))
    self.pub_grey_frame.publish(self.br.cv2_to_imgmsg(frame_grey))

  def config_updated_callback(self, msg):

    for key in msg.keys:
      if key in self.app_configuration.keys():
        self.configuration_loaded = False
        self.mask_initialised = False
        self.get_logger().info('Receiving updated configuration notification, reload')
        break

  def _setup_mask(self):
    self.get_logger().info(f"Selecting Mask Type {self.app_configuration['mask_type']}")
    self.mask = Mask.Select(self.app_configuration)

  def _load_and_validate_config(self):

      if not self.configuration_loaded:
        self._load_config()
        
        # TODO: What is the best way of exiting out of a launch script when the configuration validation fails
        valid = self._validate_config()
        if valid == False:
          self.get_logger().error('Frame Provider configuration is invalid')

        self.configuration_loaded = True

  def _load_config(self):

    self.get_logger().info(f'Loading configuration list.')

    response = self.configuration_svc.send_request(self.configuration_list)
    for config_item in response.entries:
      self.app_configuration[config_item.key] = ConfigEntryConvertor.Convert(config_item.type, config_item.value)

  def _validate_config(self):
    self.get_logger().info(f'Validating configuration.')

    valid = True

    if self.app_configuration['frame_provider_resize_frame']:
      if self.app_configuration['frame_provider_resize_dimension_h'] == None:
        self.get_logger().error('The frame_provider_resize_dimension_h config entry is null')
        valid = False
      
      if self.app_configuration['frame_provider_resize_dimension_w'] == None:
        self.get_logger().error('The frame_provider_resize_dimension_w config entry is null')
        valid = False

    if self.app_configuration['frame_provider_noise_reduction']:
      if self.app_configuration['frame_provider_blur_radius'] == None:
        self.get_logger().error('The frame_provider_blur_radius config entry is null')
        valid = False

    return valid

def main(args=None):

  rclpy.init(args=args)
  frame_provider = FrameProviderNode()
  rclpy.spin(frame_provider)
  frame_provider.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()