import os
import rclpy
import cv2
from rclpy.node import Node
from simple_tracker_interfaces.srv import Mask
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from .config_entry_convertor import ConfigEntryConvertor
from .configurations_client_async import ConfigurationsClientAsync

class MaskProviderNode(Node):

  def __init__(self):

    super().__init__('sky360_mask_provider')  

    self.configuration_list = ['mask_type', 'mask_overlay_image_file_name']
    self.app_configuration = {}
    self.configuration_loaded = False

    # setup services, publishers and subscribers
    self.configuration_svc = ConfigurationsClientAsync()
    self.mask_service = self.create_service(Mask, 'sky360/mask/image/v1', self.get_mask_callback)
    self.sub_config_updated = self.create_subscription(ConfigEntryUpdatedArray, 'sky360/config/updated/v1', self.config_updated_callback, 10)

    self.br = CvBridge()
    self.get_logger().info(f'{self.get_name()} node is up and running.')
  
  def get_mask_callback(self, request, response):

    #self.get_logger().info(f'Requesting mask {request.file_name}.')
    masks_folder = os.path.join(os.getcwd(), 'install/simple_tracker_mask_provider/share/simple_tracker_mask_provider/masks')
    mask_file_path = os.path.join(masks_folder, request.file_name)

    # TODO: This configuration update thing needs to happen in the background
    if not self.configuration_loaded:
      self._load_and_validate_config()
      self.configuration_loaded = True

    if os.path.exists(mask_file_path) == False:
      self.get_logger().error(f'Mask path {request.file_name} does not exist.')

    mask_image = cv2.imread(mask_file_path, cv2.IMREAD_GRAYSCALE)
    response.mask = self.br.cv2_to_imgmsg(mask_image)

    return response

  def config_updated_callback(self, msg):

    for key in msg.keys:
      if key in self.app_configuration.keys():
        self.configuration_loaded = False
        self.get_logger().info('Receiving updated configuration notification, reload')
        break
  
  def _load_and_validate_config(self):

      if not self.configuration_loaded:
        self._load_config()
        
        # TODO: What is the best way of exiting out of a launch script when the configuration validation fails
        valid = self._validate_config()
        if valid == False:
          self.get_logger().error('Mask configuration is invalid')

  def _load_config(self):
    #self.get_logger().info(f'Loading configuration list.')

    response = self.configuration_svc.send_request(self.configuration_list)
    for config_item in response.entries:
      self.app_configuration[config_item.key] = ConfigEntryConvertor.Convert(config_item.type, config_item.value)

  def _validate_config(self):
    #self.get_logger().info(f'Validating configuration.')

    valid = True
    # TODO: This has to be moved, we only provide the image here
    if self.app_configuration['mask_type'] == 'overlay' or self.app_configuration['mask_type'] == 'overlay_inverse':
      if self.app_configuration['mask_overlay_image_file_name'] == None:
        self.get_logger().error('The mask_overlay_image_file_name config entry is null')
        valid = False

    return valid

def main(args=None):

  rclpy.init(args=args)
  mask_provider = MaskProviderNode()
  rclpy.spin(mask_provider)
  mask_provider.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()