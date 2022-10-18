# Original work Copyright (c) 2022 Sky360
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

import datetime
import rclpy
import cv2
import time
import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import CameraFrame
from simple_tracker_shared.control_loop_node import ControlLoopNode
from simple_tracker_shared.utils import frame_resize
from .controller import Controller

class ControllerNode(ControlLoopNode):

  def __init__(self):
    super().__init__('sky360_camera')

    # setup services, publishers and subscribers
    self.pub_frame = self.create_publisher(CameraFrame, 'sky360/camera/original/v1', 10)   

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def capture_timer_callback(self):
    timer = cv2.getTickCount()
    self.success, self.frame = self.controller.read()
    self.fps = int(cv2.getTickFrequency() / (cv2.getTickCount() - timer))
    if not self.success:
      if self.app_configuration['camera_video_loop']:      
        self.controller = Controller.Select(self, self.app_configuration)  

  def control_loop(self):    
    if self.success == True:

      camera_frame_msg = CameraFrame()
      camera_frame_msg.epoch = round(time.time() * 1000) #(time.time_ns() / 1000)
      camera_frame_msg.fps = self.fps
      camera_frame_msg.frame = self.br.cv2_to_imgmsg(self.frame)

      self.pub_frame.publish(camera_frame_msg)


  def config_list(self) -> List[str]:
    return ['controller_type', 'camera_mode', 'camera_uri', 'camera_video_file', 'camera_video_loop']

  def validate_config(self) -> bool:
    valid = True

    known_controller_type = ['video', 'camera']
    if any(self.app_configuration['controller_type'] in s for s in known_controller_type) == False:
      self.get_logger().error(f'Unknown controller type ['+ self.app_configuration['controller_type']+']')
      valid = False

    if self.app_configuration['controller_type'] == 'camera':
      if self.app_configuration['camera_mode'] == None:
        self.get_logger().error('The camera_mode config entry is null')
        valid = False

      if self.app_configuration['camera_uri'] == None:
        self.get_logger().error('The camera_uri config entry is null')
        valid = False

    if self.app_configuration['controller_type'] == 'video':
      if self.app_configuration['camera_video_file'] == None:
        self.get_logger().error('The video_file config entry is null')
        valid = False
      else:
        videos_folder = os.path.join(get_package_share_directory('simple_tracker_camera'), 'videos')
        video_file_path = os.path.join(videos_folder, self.app_configuration['camera_video_file'])
        if os.path.exists(video_file_path) == False:
            self.get_logger().error(f'Could not open video {video_file_path}.')
            valid = False

    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()
      # setup timer and other helpers
      timer_period = 0.1  # seconds
      self.timer = self.create_timer(timer_period, self.capture_timer_callback)
      self.success = False
      self.frame = None
      self.fps = 0
      
    self.controller = Controller.Select(self, self.app_configuration)

def main(args=None):

  rclpy.init(args=args)
  camera = ControllerNode()
  rclpy.spin(camera)
  camera.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()