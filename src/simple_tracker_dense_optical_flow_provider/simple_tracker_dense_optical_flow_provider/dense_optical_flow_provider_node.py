import datetime
import rclpy
import cv2
from typing import List
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame
from simple_tracker_shared.control_loop_node import ControlLoopNode
from simple_tracker_shared.frame_processor import FrameProcessor
from .dense_optical_flow import DenseOpticalFlow

class DenseOpticalFlowProviderNode(ControlLoopNode):

  def __init__(self):
    super().__init__('sky360_dense_optical_flow_provider')

    # setup services, publishers and subscribers
    self.sub_grey_frame = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.grey_frame_callback, 10)
    self.pub_dense_optical_flow_frame = self.create_publisher(Frame, 'sky360/frames/dense_optical_flow/v1', 10)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def grey_frame_callback(self, msg_frame:Frame):
    self.msg_frame = msg_frame

  def control_loop(self):

    if self.msg_frame != None:

      self.frame_grey = self.br.imgmsg_to_cv2(self.msg_frame.frame)

      optical_flow_frame = self.frame_processor.process_dense_optical_flow(self.dense_optical_flow, self.frame_grey, None)

      frame_optical_flow_msg = Frame()
      frame_optical_flow_msg.epoch = self.msg_frame.epoch
      frame_optical_flow_msg.fps = self.msg_frame.fps
      frame_optical_flow_msg.frame_count = self.msg_frame.frame_count
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
      self.msg_frame:Frame = None

    self.frame_processor = FrameProcessor.Select(self.app_configuration, 'dense_optical_cuda_enable')
    self.dense_optical_flow = DenseOpticalFlow.Select(self.app_configuration)
    

def main(args=None):

  rclpy.init(args=args)
  dof_provider = DenseOpticalFlowProviderNode()
  rclpy.spin(dof_provider)
  dof_provider.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()