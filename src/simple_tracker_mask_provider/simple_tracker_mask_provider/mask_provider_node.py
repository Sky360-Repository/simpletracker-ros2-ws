import os
import rclpy
import cv2
from typing import List
from ament_index_python.packages import get_package_share_directory
from simple_tracker_interfaces.srv import Mask
from cv_bridge import CvBridge
from simple_tracker_shared.configured_node import ConfiguredNode

class MaskProviderNode(ConfiguredNode):

  def __init__(self):
    super().__init__('sky360_mask_provider')

    # setup services, publishers and subscribers
    self.mask_service = self.create_service(Mask, 'sky360/mask/image/v1', self.get_mask_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
  
  def get_mask_callback(self, request, response):

    masks_folder = self.videos_folder = os.path.join(get_package_share_directory('simple_tracker_mask_provider'), 'masks')
    mask_file_path = os.path.join(masks_folder, request.file_name)

    if os.path.exists(mask_file_path) == False:
      self.get_logger().error(f'Mask path {request.file_name} does not exist.')

    mask_image = cv2.imread(mask_file_path, cv2.IMREAD_GRAYSCALE)
    response.mask = self.br.cv2_to_imgmsg(mask_image)

    return response

  def config_list(self) -> List[str]:
    return ['mask_type', 'mask_overlay_image_file_name']

  def validate_config(self) -> bool:

    valid = True
    # TODO: This has to be moved, we only provide the image here
    if self.app_configuration['mask_type'] == 'overlay' or self.app_configuration['mask_type'] == 'overlay_inverse':
      if self.app_configuration['mask_overlay_image_file_name'] == None:
        self.get_logger().error('The mask_overlay_image_file_name config entry is null')
        valid = False

    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()

def main(args=None):

  rclpy.init(args=args)
  mask_provider = MaskProviderNode()
  rclpy.spin(mask_provider)
  mask_provider.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()