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

import os
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.config_entry_convertor import ConfigEntryConvertor
from simple_tracker_interfaces.msg import ConfigItem
from .mask_client_async import MaskClientAsync

from cv_bridge import CvBridge
import cv2
 
class KeyHandler():

  def __init__(self, node: ConfiguredNode, br: CvBridge):
    self.node = node
    self.br = CvBridge()
    self.mask_svc = MaskClientAsync()
    self.init_state()    

  def handle_key_press(self, k):

    if k > 0:
      self.node.get_logger().info(f'Key press {k} captured.')
      if k == 43: # +
        self.increase_frame_size()
      elif k == 45: # -
        self.decrease_frame_size()
      elif k == 99: # c
        self.update_controller()        
      elif k == 109: # m
        self.update_mask()


  def update_mask(self):

    if self.state['mask_overlay_image_file_name'] == 'mask-shrubs-inverse-overlay.jpg':

      mask_file_path = '/workspaces/simpletracker-ros2-ws/beeks_mask.jpg'

      if os.path.exists(mask_file_path) == False:
        self.node.get_logger().error(f'Mask path {mask_file_path} does not exist.')

      mask_image = cv2.imread(mask_file_path, cv2.IMREAD_GRAYSCALE)
      msg_mask_image = self.br.cv2_to_imgmsg(mask_image)    
      mask_type = 'overlay_inverse'
      file_name = 'beeks_mask.jpg'

      response = self.mask_svc.send_request(msg_mask_image, mask_type,file_name)
      if response.success:
        self.node.get_logger().info(f'Mask update response: {response}')
        self.state['mask_overlay_image_file_name'] = 'beeks_mask.jpg'
        self.state['mask_type'] = 'overlay_inverse'

    else:
      mask_type_config = ConfigItem()
      mask_type_config.key = 'mask_type'
      mask_type_config.type = 'str'
      mask_type_config.value = 'overlay_inverse'

      mask_image_config = ConfigItem()
      mask_image_config.key = 'mask_overlay_image_file_name'
      mask_image_config.type = 'str'
      mask_image_config.value = 'mask-shrubs-inverse-overlay.jpg'

      config_array = [mask_image_config, mask_type_config]
      self.handle_config_update(config_array)

  def update_controller(self):

    controller_type_config = ConfigItem()

    if self.state['controller_type'] == 'video':
      controller_type_config.key = 'controller_type'
      controller_type_config.type = 'str'
      controller_type_config.value = 'camera'
    else: 
      controller_type_config = ConfigItem()
      controller_type_config.key = 'controller_type'
      controller_type_config.type = 'str'
      controller_type_config.value = 'video'

    config_array = [controller_type_config]
    self.handle_config_update(config_array)

  def increase_frame_size(self):

    f_dimension_h_config = ConfigItem()
    f_dimension_w_config = ConfigItem()
    v_dimension_h_config = ConfigItem()
    v_dimension_w_config = ConfigItem()

    f_dimension_h_config.key = 'frame_provider_resize_dimension_h'
    f_dimension_h_config.type = 'int'
    f_dimension_h_config.value = str(self.state['frame_provider_resize_dimension_h'] + 40)

    f_dimension_w_config.key = 'frame_provider_resize_dimension_w'
    f_dimension_w_config.type = 'int'
    f_dimension_w_config.value = str(self.state['frame_provider_resize_dimension_w'] + 40)

    v_dimension_h_config.key = 'visualiser_resize_dimension_h'
    v_dimension_h_config.type = 'int'
    v_dimension_h_config.value = str(self.state['visualiser_resize_dimension_h'] + 40)

    v_dimension_w_config.key = 'visualiser_resize_dimension_w'
    v_dimension_w_config.type = 'int'
    v_dimension_w_config.value = str(self.state['visualiser_resize_dimension_w'] + 40)

    config_array = [f_dimension_h_config, f_dimension_w_config, v_dimension_h_config, v_dimension_w_config]
    self.handle_config_update(config_array)
  
  def decrease_frame_size(self):

    f_dimension_h_config = ConfigItem()
    f_dimension_w_config = ConfigItem()
    v_dimension_h_config = ConfigItem()
    v_dimension_w_config = ConfigItem()

    f_dimension_h_config.key = 'frame_provider_resize_dimension_h'
    f_dimension_h_config.type = 'int'
    f_dimension_h_config.value = str(self.state['frame_provider_resize_dimension_h'] - 40)

    f_dimension_w_config.key = 'frame_provider_resize_dimension_w'
    f_dimension_w_config.type = 'int'
    f_dimension_w_config.value = str(self.state['frame_provider_resize_dimension_w'] - 40)

    v_dimension_h_config.key = 'visualiser_resize_dimension_h'
    v_dimension_h_config.type = 'int'
    v_dimension_h_config.value = str(self.state['visualiser_resize_dimension_h'] - 40)

    v_dimension_w_config.key = 'visualiser_resize_dimension_w'
    v_dimension_w_config.type = 'int'
    v_dimension_w_config.value = str(self.state['visualiser_resize_dimension_w'] - 40)

    config_array = [f_dimension_h_config, f_dimension_w_config, v_dimension_h_config, v_dimension_w_config]
    self.handle_config_update(config_array)


  def handle_config_update(self, config_array):
    update_result = self.node.configuration_svc.send_update_config_request(config_array)
    if update_result.success:
      for config in config_array:
        self.node.get_logger().info(f'{config.key} was updated successfully to {config.value}.')
        self.state[config.key] = ConfigEntryConvertor.Convert(config.type, config.value)
    else:
      self.node.get_logger().warn(f'Error updating configuration: {update_result.message}.')

  def init_state(self):
    self.state = {}

    self.state['mask_overlay_image_file_name'] = 'mask-shrubs-inverse-overlay.jpg'
    self.state['mask_type'] = 'overlay_inverse'

    self.state['controller_type'] = 'video'

    self.state['frame_provider_resize_dimension_h'] = 960
    self.state['frame_provider_resize_dimension_w'] = 960
    self.state['visualiser_resize_dimension_h'] = 960
    self.state['visualiser_resize_dimension_w'] = 960
    
