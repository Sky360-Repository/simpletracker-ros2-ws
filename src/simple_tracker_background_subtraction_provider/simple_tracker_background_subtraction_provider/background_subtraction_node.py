import datetime
import rclpy
import cv2
from typing import List
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame
from simple_tracker_shared.control_loop_node import ControlLoopNode
from .background_subtractor import BackgroundSubtractor

class BackgroundSubtractionProviderNode(ControlLoopNode):

  def __init__(self):
    super().__init__('sky360_foreground_mask_provider')

    # setup services, publishers and subscribers
    self.sub_grey_frame = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.grey_frame_callback, 10)
    self.pub_foreground_mask_frame = self.create_publisher(Frame, 'sky360/frames/foreground_mask/v1', 10)
    self.pub_masked_background_frame = self.create_publisher(Frame, 'sky360/frames/masked_background/v1', 10)    

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def grey_frame_callback(self, msg_frame:Frame):
    self.msg_frame = msg_frame

  def control_loop(self):

    if self.msg_frame != None:

      frame_grey = self.br.imgmsg_to_cv2(self.msg_frame.frame)

      foreground_mask_frame = self.background_subtractor.apply(frame_grey) #, learningRate=self.background_subtractor_learning_rate)
      frame_masked_background = cv2.bitwise_and(frame_grey, frame_grey, mask=foreground_mask_frame)

      frame_foreground_mask_msg = Frame()
      frame_foreground_mask_msg.epoch = self.msg_frame.epoch
      frame_foreground_mask_msg.fps = self.msg_frame.fps
      frame_foreground_mask_msg.frame_count = self.msg_frame.frame_count
      frame_foreground_mask_msg.frame = self.br.cv2_to_imgmsg(foreground_mask_frame)
      self.pub_foreground_mask_frame.publish(frame_foreground_mask_msg)

      frame_masked_background_msg = Frame()
      frame_masked_background_msg.epoch = self.msg_frame.epoch
      frame_masked_background_msg.fps = self.msg_frame.fps
      frame_masked_background_msg.frame_count = self.msg_frame.frame_count
      frame_masked_background_msg.frame = self.br.cv2_to_imgmsg(frame_masked_background)
      self.pub_masked_background_frame.publish(frame_masked_background_msg)

  def config_list(self) -> List[str]:
    return ['background_subtractor_sensitivity', 'background_subtractor_type', 'background_subtractor_cuda_enable']

  def validate_config(self) -> bool:
    valid = True

    background_subtractor_type = self.app_configuration['background_subtractor_type']
    supported_bgsubtractors = {'KNN', 'MOG', 'MOG2', 'BGS_FD', 'BGS_SFD', 'BGS_WMM', 'BGS_WMV', 'BGS_ABL', 'BGS_ASBL', 'BGS_MOG2', 'BGS_PBAS', 'BGS_SD', 'BGS_SuBSENSE', 'BGS_LOBSTER', 'BGS_PAWCS', 'BGS_TP', 'BGS_VB', 'BGS_CB'}
    supported_cuda_bgsubtractors = {'MOG2_CUDA', 'MOG_CUDA'}
    supported = False
    if self.app_configuration['background_subtractor_cuda_enable']:
        supported = background_subtractor_type in supported_cuda_bgsubtractors
    else:
        supported = background_subtractor_type in supported_bgsubtractors

    if supported == False:
      self.get_logger().error(f"Unknown background subtractor type ({background_subtractor_type}) when cuda-enabled: {self.app_configuration['background_subtractor_cuda_enable']}.")
      valid = False

    if self.app_configuration['background_subtractor_sensitivity'] == None:
      self.get_logger().error('The background_subtractor_sensitivity config entry is null')
      valid = False
      
    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.msg_frame: Frame = None
      self.br = CvBridge()

    self.background_subtractor = BackgroundSubtractor.Select(self.app_configuration)

def main(args=None):

  rclpy.init(args=args)
  background_subtractor_provider = BackgroundSubtractionProviderNode()
  rclpy.spin(background_subtractor_provider)
  background_subtractor_provider.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()