import datetime
import rclpy
import cv2
import time
from typing import List
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import CameraFrame
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.utils import frame_resize

class CameraNode(ConfiguredNode):

  def __init__(self):
    super().__init__('sky360_camera')

    # setup services, publishers and subscribers
    self.pub_frame = self.create_publisher(CameraFrame, 'sky360/camera/original/v1', 10)   

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def capture_timer_callback(self):

    timer = cv2.getTickCount()
    success, frame = self.capture.read()
    if success == True:

      if self.app_configuration['camera_resize_frame']:
        frame = frame_resize(frame, height=self.app_configuration['camera_resize_dimension_h'], width=self.app_configuration['camera_resize_dimension_w'])

      camera_frame_msg = CameraFrame()
      camera_frame_msg.epoch = round(time.time() * 1000) #(time.time_ns() / 1000)
      camera_frame_msg.fps = int(cv2.getTickFrequency() / (cv2.getTickCount() - timer))
      camera_frame_msg.frame = self.br.cv2_to_imgmsg(frame)

      self.pub_frame.publish(camera_frame_msg)

  def config_list(self) -> List[str]:
    return ['camera_mode', 'camera_uri', 'camera_resize_dimension_h', 'camera_resize_dimension_w', 'camera_resize_frame', 'camera_cuda_enable']

  def validate_config(self) -> bool:
    valid = True

    if self.app_configuration['camera_uri'] == None:
      self.get_logger().error('The camera_uri config entry is null')
      valid = False

    if self.app_configuration['camera_resize_frame']:
      if self.app_configuration['camera_resize_dimension_h'] is None and self.app_configuration['camera_resize_dimension_w'] is None:
        self.get_logger().error('Both camera_resize_dimension_h and camera_resize_dimension_w config entries are null')
        valid = False

    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()
      # setup timer and other helpers
      timer_period = 0.1  # seconds
      self.timer = self.create_timer(timer_period, self.capture_timer_callback)
      
    self.capture = cv2.VideoCapture(self.app_configuration['camera_uri'])


def main(args=None):

  rclpy.init(args=args)
  camera = CameraNode()
  rclpy.spin(camera)
  camera.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()