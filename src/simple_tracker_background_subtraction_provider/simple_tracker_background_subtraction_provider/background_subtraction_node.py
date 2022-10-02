import datetime
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from simple_tracker_shared.config_entry_convertor import ConfigEntryConvertor
from simple_tracker_shared.configurations_client_async import ConfigurationsClientAsync
from .background_subtractor import BackgroundSubtractor

class BackgroundSubtractionProviderNode(Node):

  def __init__(self):

    super().__init__('sky360_foreground_mask_provider')  

    self.configuration_list = ['tracker_detection_sensitivity', 'background_subtractor_cuda_enable']
    self.app_configuration = {}
    self.configuration_loaded = False

    # setup services, publishers and subscribers
    self.configuration_svc = ConfigurationsClientAsync()
    self.sub_grey_frame = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.grey_frame_callback, 10)
    self.pub_foreground_mask_frame = self.create_publisher(Frame, 'sky360/frames/foreground_mask/v1', 10)
    self.pub_masked_background_frame = self.create_publisher(Frame, 'sky360/frames/masked_background/v1', 10)
    self.sub_config_updated = self.create_subscription(ConfigEntryUpdatedArray, 'sky360/config/updated/v1', self.config_updated_callback, 10)

    # setup timer and other helpers
    self.br = CvBridge()

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def grey_frame_callback(self, data:Frame):

    # TODO: This configuration update thing needs to happen in the background
    if not self.configuration_loaded:
      self._load_and_validate_config()
      self.configuration_loaded = True

    frame_grey = self.br.imgmsg_to_cv2(data.frame)

    foreground_mask_frame = self.background_subtractor.apply(frame_grey) #, learningRate=self.background_subtractor_learning_rate)
    frame_masked_background = cv2.bitwise_and(frame_grey, frame_grey, mask=foreground_mask_frame)

    frame_foreground_mask_msg = Frame()
    frame_foreground_mask_msg.epoch = data.epoch
    frame_foreground_mask_msg.fps = data.fps
    frame_foreground_mask_msg.frame_count = data.frame_count
    frame_foreground_mask_msg.frame = self.br.cv2_to_imgmsg(foreground_mask_frame)
    self.pub_foreground_mask_frame.publish(frame_foreground_mask_msg)

    frame_masked_background_msg = Frame()
    frame_masked_background_msg.epoch = data.epoch
    frame_masked_background_msg.fps = data.fps
    frame_masked_background_msg.frame_count = data.frame_count
    frame_masked_background_msg.frame = self.br.cv2_to_imgmsg(frame_masked_background)
    self.pub_masked_background_frame.publish(frame_masked_background_msg)

  def config_updated_callback(self, msg:ConfigEntryUpdatedArray):

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
          self.get_logger().error('Frame Provider configuration is invalid')

        self.background_subtractor = BackgroundSubtractor.Select(self.app_configuration)

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
  background_subtractor_provider = BackgroundSubtractionProviderNode()
  rclpy.spin(background_subtractor_provider)
  background_subtractor_provider.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()