import datetime
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from .config_entry_convertor import ConfigEntryConvertor
from .configurations_client_async import ConfigurationsClientAsync
from .dense_optical_flow import DenseOpticalFlow
#from simple_tracker_shared.simple_tracker_shared import ConfigEntryConvertor
#from simple_tracker_shared.simple_tracker_shared import ConfigurationsClientAsync

class DenseOpticalFlowProviderNode(Node):

  def __init__(self):

    super().__init__('sky360_dense_optical_flow_provider')  

    self.configuration_list = ['dense_optical_flow_h', 'dense_optical_flow_w', 'dense_optical_cuda_enable']
    self.app_configuration = {}
    self.configuration_loaded = False

    # setup services, publishers and subscribers
    self.configuration_svc = ConfigurationsClientAsync()
    self.sub_grey_frame = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.grey_frame_callback, 10)
    self.pub_dense_optical_flow_frame = self.create_publisher(Frame, 'sky360/frames/dense_optical_flow/v1', 10)
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

    optical_flow_frame = self.dense_optical_flow.process_grey_frame(frame_grey)

    #gpu_frame_grey = cv2.cuda_GpuMat()
    #gpu_frame_grey.upload(frame_grey, stream=None) 
    #optical_flow_frame = self.dense_optical_flow.process_grey_frame(gpu_frame_grey)

    frame_optical_flow_msg = Frame()
    frame_optical_flow_msg.frame_count = data.frame_count
    frame_optical_flow_msg.frame = self.br.cv2_to_imgmsg(optical_flow_frame)
    self.pub_dense_optical_flow_frame.publish(frame_optical_flow_msg)

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

        self.dense_optical_flow = DenseOpticalFlow.Select(self.app_configuration)

        self.configuration_loaded = True

  def _load_config(self):
    #self.get_logger().info(f'Loading configuration list.')

    response = self.configuration_svc.send_request(self.configuration_list)
    for config_item in response.entries:
      self.app_configuration[config_item.key] = ConfigEntryConvertor.Convert(config_item.type, config_item.value)

  def _validate_config(self):
    #self.get_logger().info(f'Validating configuration.')

    valid = True

    if self.app_configuration['dense_optical_flow_h'] == None:
      self.get_logger().error('The dense_optical_flow_h config entry is null')
      valid = False
      
    if self.app_configuration['dense_optical_flow_w'] == None:
      self.get_logger().error('The dense_optical_flow_w config entry is null')
      valid = False

    return valid

def main(args=None):

  rclpy.init(args=args)
  dof_provider = DenseOpticalFlowProviderNode()
  rclpy.spin(dof_provider)
  dof_provider.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()