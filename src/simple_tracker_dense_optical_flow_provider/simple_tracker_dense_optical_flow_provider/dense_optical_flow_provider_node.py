import datetime
import rclpy
import cv2
from typing import List
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame
from simple_tracker_shared.configured_node import ConfiguredNode
from .dense_optical_flow import DenseOpticalFlow

class DenseOpticalFlowProviderNode(ConfiguredNode):

  def __init__(self):
    super().__init__('sky360_dense_optical_flow_provider')

    # setup services, publishers and subscribers
    self.sub_grey_frame = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.grey_frame_callback, 10)
    self.pub_dense_optical_flow_frame = self.create_publisher(Frame, 'sky360/frames/dense_optical_flow/v1', 10)

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
    frame_optical_flow_msg.epoch = data.epoch
    frame_optical_flow_msg.fps = data.fps
    frame_optical_flow_msg.frame_count = data.frame_count
    frame_optical_flow_msg.frame = self.br.cv2_to_imgmsg(optical_flow_frame)
    self.pub_dense_optical_flow_frame.publish(frame_optical_flow_msg)

  def config_list(self) -> List[str]:
    return ['dense_optical_flow_h', 'dense_optical_flow_w', 'dense_optical_cuda_enable']

  def validate_config(self) -> bool:
    valid = True

    if self.app_configuration['dense_optical_flow_h'] == None:
      self.get_logger().error('The dense_optical_flow_h config entry is null')
      valid = False
      
    if self.app_configuration['dense_optical_flow_w'] == None:
      self.get_logger().error('The dense_optical_flow_w config entry is null')
      valid = False

    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()

    self.dense_optical_flow = DenseOpticalFlow.Select(self.app_configuration)
    

def main(args=None):

  rclpy.init(args=args)
  dof_provider = DenseOpticalFlowProviderNode()
  rclpy.spin(dof_provider)
  dof_provider.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()