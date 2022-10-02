import rclpy
from abc import abstractmethod
from typing import List
from rclpy.node import Node as BaseNode
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from simple_tracker_shared.config_entry_convertor import ConfigEntryConvertor
from simple_tracker_shared.configurations_client_async import ConfigurationsClientAsync

class ConfiguredNode(BaseNode):

  def __init__(self, node_name: str):
    super().__init__(node_name)

    self.app_configuration = {}
    self.configuration_svc = ConfigurationsClientAsync()
    self.sub_config_updated = self.create_subscription(ConfigEntryUpdatedArray, 'sky360/config/updated/v1', self.config_updated_callback, 10)

    self.configuration_loaded = False
    if not self.configuration_loaded:
      self.load_and_validate_config()
      self.configuration_loaded = True

  def config_updated_callback(self, msg:ConfigEntryUpdatedArray):
    for key in msg.keys:
      if key in self.app_configuration.keys():
        self.configuration_loaded = False
        self.get_logger().info('Receiving updated configuration notification, reload')
        break

  def load_and_validate_config(self):

      if not self.configuration_loaded:
        self.load_config()
        
        # TODO: What is the best way of exiting out of a launch script when the configuration validation fails
        valid = self.validate_config()
        if valid == False:
          self.get_logger().error( f'{self.get_name()} configuration is invalid')

        self.configuration_loaded = True

  def load_config(self):
    #self.get_logger().info(f'Loading configuration list.')
    response = self.configuration_svc.send_request(self.config_list())
    for config_item in response.entries:
      self.app_configuration[config_item.key] = ConfigEntryConvertor.Convert(config_item.type, config_item.value)

  @abstractmethod
  def config_list(self) -> List[str]:
    pass

  @abstractmethod
  def validate_config(self) -> bool:
    pass