import datetime
import rclpy
import cv2
import time
import os
from typing import List
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import CameraFrame
from simple_tracker_shared.utils import frame_resize
from simple_tracker_shared.configured_node import ConfiguredNode

class VideoNode(ConfiguredNode):

  def __init__(self):

    super().__init__('sky360_video')  

    # setup services, publishers and subscribers
    self.pub_frame = self.create_publisher(CameraFrame, 'sky360/camera/original/v1', 10)    

    # setup timer and other helpers
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.capture_timer_callback)
    self.br = CvBridge()
    self.broad_casting = False

    self.videos_folder = os.path.join(os.getcwd(), 'install/simple_tracker_video/share/simple_tracker_video/videos')

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def capture_timer_callback(self):

    if not self.broad_casting:
      self.video_file_path = os.path.join(self.videos_folder, self.app_configuration['video_file'])
      self.capture = cv2.VideoCapture(self.video_file_path)
      if not self.capture.isOpened():
        print("Could not open video")
        self.get_logger().error(f'Could not open video {self.video_file_path}.')
      self.broad_casting = True

    timer = cv2.getTickCount()
    success, frame = self.capture.read()
    if success == True:

      if self.app_configuration['video_resize_frame']:
        frame = frame_resize(frame, height=self.app_configuration['video_resize_dimension_h'], width=self.app_configuration['video_resize_dimension_w'])

      camera_frame_msg = CameraFrame()
      camera_frame_msg.epoch = round(time.time() * 1000) #(time.time_ns() / 1000)
      camera_frame_msg.fps = int(cv2.getTickFrequency() / (cv2.getTickCount() - timer))
      camera_frame_msg.frame = self.br.cv2_to_imgmsg(frame)

      self.pub_frame.publish(camera_frame_msg)

    else:
      if self.app_configuration['video_loop']:      
        self.broad_casting = False

  def config_list(self) -> List[str]:
    return ['video_file', 'video_loop', 'video_resize_frame', 'video_resize_dimension_h', 'video_resize_dimension_w', 'video_cuda_enable']

  def validate_config(self) -> bool:
    valid = True

    if self.app_configuration['video_file'] == None:
      self.get_logger().error('The video_file config entry is null')
      valid = False

    if self.app_configuration['video_resize_frame']:
      if self.app_configuration['video_resize_dimension_h'] is None and self.app_configuration['video_resize_dimension_w'] is None:
        self.get_logger().error('Both video_resize_dimension_h and video_resize_dimension_w config entries are null')
        valid = False

    return valid 

def main(args=None):

  rclpy.init(args=args)
  video = VideoNode()
  rclpy.spin(video)
  video.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()