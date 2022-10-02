import datetime
import rclpy
import cv2
from typing import List
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame
from simple_tracker_shared.configured_node import ConfiguredNode
from .background_subtractor import BackgroundSubtractor

class BackgroundSubtractionProviderNode(ConfiguredNode):

  def __init__(self):

    super().__init__('sky360_foreground_mask_provider')  

    # setup services, publishers and subscribers
    self.sub_grey_frame = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.grey_frame_callback, 10)
    self.pub_foreground_mask_frame = self.create_publisher(Frame, 'sky360/frames/foreground_mask/v1', 10)
    self.pub_masked_background_frame = self.create_publisher(Frame, 'sky360/frames/masked_background/v1', 10)

    # setup timer and other helpers
    self.br = CvBridge()
    self.background_subtractor = BackgroundSubtractor.Select(self.app_configuration)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def config_list(self) -> List[str]:
    return ['tracker_detection_sensitivity', 'background_subtractor_cuda_enable']

  def validate_config(self) -> bool:
    valid = True

    if self.app_configuration['tracker_detection_sensitivity'] == None:
      self.get_logger().error('The tracker_detection_sensitivity config entry is null')
      valid = False
      
    return valid

  def grey_frame_callback(self, data:Frame):

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

def main(args=None):

  rclpy.init(args=args)
  background_subtractor_provider = BackgroundSubtractionProviderNode()
  rclpy.spin(background_subtractor_provider)
  background_subtractor_provider.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()