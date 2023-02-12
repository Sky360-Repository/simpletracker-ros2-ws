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

import traceback as tb
import os
import rclpy
import cv2
from typing import List
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from simple_tracker_interfaces.srv import Mask, MaskUpdate
from simple_tracker_interfaces.msg import ConfigItem
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner

class MaskProviderNode(ConfiguredNode):

  def __init__(self):
    super().__init__('sky360_mask_provider')

    # setup services, publishers and subscribers
    self.get_mask_service = self.create_service(Mask, 'sky360/mask/image', self.get_mask_callback)
    self.put_mask_service = self.create_service(MaskUpdate, 'sky360/mask/update', self.put_mask_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
  
  def get_mask_callback(self, request, response):

    try:
      masks_folder = self.videos_folder = os.path.join(get_package_share_directory('simple_tracker_mask_provider'), 'masks')
      mask_file_path = os.path.join(masks_folder, request.file_name)

      if os.path.exists(mask_file_path) == False:
        self.get_logger().error(f'Mask path {request.file_name} does not exist.')

      mask_image = cv2.imread(mask_file_path, cv2.IMREAD_GRAYSCALE)
      response.mask = self.br.cv2_to_imgmsg(mask_image)
    except Exception as e:
      self.get_logger().error(f"Exception getting mask. Error: {e}.")
      self.get_logger().error(tb.format_exc())

    return response

  def put_mask_callback(self, request, response):

    try:
      self.write_mask_file(request, response)
    except Exception as e:
      self.get_logger().error(f"Exception during writing mask file. Error: {e}.")

    return response

  def write_mask_file(self, request, response):

    mask_image = self.br.imgmsg_to_cv2(request.mask)
    masks_folder = self.videos_folder = os.path.join(get_package_share_directory('simple_tracker_mask_provider'), 'masks')
    mask_file_path = os.path.join(masks_folder, request.file_name)

    if os.path.exists(mask_file_path) == False:
      self.get_logger().info(f'Mask path {request.file_name} does not exist, adding mask.')
    else:
      self.get_logger().info(f'Mask path {request.file_name} does exist, overwriting.')

    response.success = cv2.imwrite(mask_file_path, mask_image)

    if response.success:
      # update config 
      mask_type_config = ConfigItem()
      mask_type_config.key = 'mask_type'
      mask_type_config.type = 'str'
      mask_type_config.value = request.mask_type

      mask_image_config = ConfigItem()
      mask_image_config.key = 'mask_overlay_image_file_name'
      mask_image_config.type = 'str'
      mask_image_config.value = request.file_name

      config_array = [mask_image_config, mask_type_config]
      update_result = self.configuration_svc.send_update_config_request(config_array)
      if update_result.success:
        self.get_logger().info(f'Mask image was updated successfully to {request.file_name}.')
      else:
        self.get_logger().info(f'Mask image config update, failed. [{update_result.message}]')

    return response

  def config_list(self) -> List[str]:
    return ['mask_type', 'mask_overlay_image_file_name']

  def validate_config(self) -> bool:

    valid = True
    # TODO: This has to be moved, we only provide the image here
    self.image_masks = ['overlay', 'overlay_inverse']
    if any(self.app_configuration['mask_type'] in s for s in self.image_masks):  
      if self.app_configuration['mask_overlay_image_file_name'] == None:
        self.get_logger().error('The mask_overlay_image_file_name config entry is null')
        valid = False

    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()

def main(args=None):

  rclpy.init(args=args)

  node = MaskProviderNode()

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()