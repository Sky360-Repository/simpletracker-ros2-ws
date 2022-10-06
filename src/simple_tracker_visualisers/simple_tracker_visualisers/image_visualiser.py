import rclpy
from typing import List
from simple_tracker_interfaces.msg import Frame
from simple_tracker_interfaces.msg import CameraFrame
from simple_tracker_interfaces.msg import TrackingState
from simple_tracker_interfaces.msg import TrackArray
from simple_tracker_interfaces.msg import Track
from simple_tracker_interfaces.msg import BoundingBox
from simple_tracker_shared.configured_node import ConfiguredNode
from cv_bridge import CvBridge
import cv2
 
class ImageVisualiserNode(ConfiguredNode):

  PROVISIONARY_TARGET = 1
  ACTIVE_TARGET = 2
  LOST_TARGET = 3

  def __init__(self):
    super().__init__('image_visualiser_node')

    #self.camera_original_sub = self.create_subscription(CameraFrame, 'sky360/camera/original/v1', self.camera_original_callback, 10)
    #self.fp_original_sub = self.create_subscription(Frame, 'sky360/frames/original/v1', self.fp_original_callback, 10)
    #self.fp_original_masked_sub = self.create_subscription(Frame, 'sky360/frames/masked/v1', self.fp_original_masked_callback, 10)
    #self.fp_grey_sub = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.fp_grey_callback, 10)
    #self.dof_sub = self.create_subscription(Frame, 'sky360/frames/dense_optical_flow/v1', self.dof_callback, 10)
    #self.forground_sub = self.create_subscription(Frame, 'sky360/frames/foreground_mask/v1', self.foreground_callback, 10)
    #self.masked_background_sub = self.create_subscription(Frame, 'sky360/frames/masked_background/v1', self.masked_background_callback, 10)
    #self.tracking_state_sub = self.create_subscription(TrackingState, 'sky360/tracker/tracking_state/v1', self.tracking_state_callback, 10)    
    self.fp_annotated_sub = self.create_subscription(Frame, 'sky360/frames/annotated/v1', self.fp_annotated_callback, 10)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_original_callback(self, data:CameraFrame):
    camera_original_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("camera/original", camera_original_frame)
    cv2.waitKey(1)

  def fp_original_callback(self, data:Frame):
    original_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("fp/original", original_frame)
    cv2.waitKey(1)

  def fp_original_masked_callback(self, data:Frame):
    masked_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("fp/masked", masked_frame)
    cv2.waitKey(1)

  def fp_grey_callback(self, data:Frame):
    camera_grey_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("fp/grey", camera_grey_frame)
    cv2.waitKey(1)

  def dof_callback(self, data:Frame):
    dof_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("dense-optical-flow", dof_frame)
    cv2.waitKey(1)

  def foreground_callback(self, data:Frame):
    foreground_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("foreground-mask", foreground_frame)
    cv2.waitKey(1)

  def masked_background_callback(self, data:Frame):
    masked_background_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("masked-background", masked_background_frame)
    cv2.waitKey(1)

  def tracking_state_callback(self, data:TrackingState):
    if self.configuration_loaded == False:
      pass

    msg = f"(Sky360) Tracker Status: trackable:{data.trackable}, alive:{data.alive}, started:{data.started}, ended:{data.ended}, frame count:{data.frame_count}, frame epoch:{data.epoch}, fps:{data.fps} "
    if self.app_configuration['visualiser_log_status_to_console']:
      self.get_logger().info(msg)

  def fp_annotated_callback(self, data:Frame):
    annotated_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("fp/annotated", annotated_frame)
    cv2.waitKey(1)

  def config_list(self) -> List[str]:
    return ['visualiser_font_size', 'visualiser_font_thickness', 'visualiser_bbox_line_thickness', 'visualiser_bbox_size', 
      'visualiser_log_status_to_console']

  def validate_config(self) -> bool:
    valid = True

    if self.app_configuration['visualiser_font_size'] == None:
      self.get_logger().error('The visualiser_font_size config entry is null')
      valid = False

    if self.app_configuration['visualiser_font_thickness'] == None:
      self.get_logger().error('The visualiser_font_thickness config entry is null')
      valid = False

    return valid  

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()
      self.font_colour = (50, 170, 50)

    self.font_size = self.app_configuration['visualiser_font_size']
    self.font_thickness = self.app_configuration['visualiser_font_thickness']
    self.bbox_line_thickness = self.app_configuration['visualiser_bbox_line_thickness']


def main(args=None):

  rclpy.init(args=args)
  image_visualiser = ImageVisualiserNode()
  rclpy.spin(image_visualiser)
  image_visualiser.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()