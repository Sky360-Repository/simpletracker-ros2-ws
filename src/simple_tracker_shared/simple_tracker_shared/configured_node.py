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


import rclpy
from abc import abstractmethod
from typing import List
from rclpy.node import Node as BaseNode
from rclpy.qos import QoSProfile, QoSPresetProfiles, qos_profile_services_default, qos_profile_sensor_data
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from simple_tracker_shared.config_entry_convertor import ConfigEntryConvertor
from simple_tracker_shared.configurations_client_async import ConfigurationsClientAsync

class ConfiguredNode(BaseNode):

  def __init__(self, node_name: str):
    super().__init__(node_name)

    self.app_configuration = {}

    self.configuration_svc = ConfigurationsClientAsync()
    self.sub_config_updated = self.create_subscription(ConfigEntryUpdatedArray, 'sky360/config/updated', 
      self.config_updated_callback, qos_profile_sensor_data)

    self.configuration_loaded = self.load_and_validate_config(None)

  def config_updated_callback(self, msg:ConfigEntryUpdatedArray):
    for key in msg.keys:
      if key in self.app_configuration.keys():
        self.configuration_loaded = False
        self.get_logger().info('Receiving updated configuration notification, reload')
        break
    
    if self.configuration_loaded == False:
      self.load_and_validate_config(msg.keys)
      self.configuration_loaded = True

  def load_and_validate_config(self, keys) -> bool:
      self.load_config()        
      # TODO: What is the best way of exiting out of a launch script when the configuration validation fails
      valid = self.validate_config()
      if valid:
        if keys is None:
          self.on_config_loaded(init = True)
        else:
          for key in keys:
            if any(key in s for s in self.config_list()):
              self.on_config_loaded(init = False)
              break
      else:
        self.get_logger().error( f'{self.get_name()} configuration is invalid')
        
      return valid

  def load_config(self):
    #self.get_logger().info(f'Loading configuration list.')
    response = self.configuration_svc.send_get_config_request(self.config_list())
    for config_item in response.entries:
      self.app_configuration[config_item.key] = ConfigEntryConvertor.Convert(config_item.type, config_item.value)

  @abstractmethod
  def config_list(self) -> List[str]:
    pass

  @abstractmethod
  def validate_config(self) -> bool:
    pass

  @abstractmethod
  def on_config_loaded(self, init: bool):
    pass