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

import cv2
import os
import numpy as np
from simple_tracker_shared.control_loop_node import ConfiguredNode
from simple_tracker_shared.config_entry_convertor import ConfigEntryConvertor
from simple_tracker_interfaces.msg import ConfigItem
from ament_index_python.packages import get_package_share_directory
from simple_tracker_shared.utils import frame_resize, get_optimal_font_scale

class SimulationTestCase():

  def __init__(self, node: ConfiguredNode, tests, dimension, file_name=None):
    self.node = node
    self.logger = node.get_logger()
    self.tests = tests
    self.running = True
    (self.w, self.h) = dimension
    self.frame = None

    if not file_name is None and len(file_name) > 0:
      frames_folder = self.videos_folder = os.path.join(get_package_share_directory('simulated_video_provider'), 'still_frames')
      frame_file_path = os.path.join(frames_folder, self.file_name)

      if os.path.exists(frame_file_path):
        self.frame = cv2.imread(frame_file_path, cv2.IMREAD_COLOR)
        self.frame = frame_resize(self.frame, width=self.w, height=self.h)
      else:
        self.logger.info(f'Still frame path {frame_file_path} does not exist.')      

    if self.frame is None:
      self.frame = np.full((self.h, self.w, 3) , (255, 255, 255), np.uint8)
      
    simulation_message = f"(Sky360) Simulation H:{self.h}, W:{self.w}"
    fontScale = get_optimal_font_scale(simulation_message, int(self.w/2))
    cv2.putText(self.frame, simulation_message, (25, int(self.h-25)), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (50, 170, 50), 2)

  @property
  def active(self):
    return self.running

  @property
  def image(self):
    return self.frame

  def run(self):
    if self.running:
      running = False
      frame_synthetic = self.frame.copy()

      for test in self.tests:
        if test.running:
          test.run(frame_synthetic)
          running = True
    
      self.running = running
      if self.running:
        return frame_synthetic

    return self.frame

  def notify_of_frame_change(self):

    # MikeG: We do this hear so that other nodes that have instantiated objects based on frame size, can refresh their
    # objects. Certain objects (bgs subtractor, mask, blob detector etc) do not deal with frame resizes well

    f_dimension_h_config = ConfigItem()
    f_dimension_w_config = ConfigItem()

    f_dimension_h_config.key = 'frame_provider_resize_dimension_h'
    f_dimension_h_config.type = 'int'
    f_dimension_h_config.value = str(self.h)

    f_dimension_w_config.key = 'frame_provider_resize_dimension_w'
    f_dimension_w_config.type = 'int'
    f_dimension_w_config.value = str(self.w)

    config_array = [f_dimension_h_config, f_dimension_w_config]
    self.handle_config_update(config_array)
  
  def handle_config_update(self, config_array):
    update_result = self.node.configuration_svc.send_update_config_request(config_array)
    if update_result.success:
      for config in config_array:
        self.node.get_logger().info(f'{config.key} was updated successfully to {config.value}.')
    else:
      self.node.get_logger().warn(f'Error updating configuration: {update_result.message}.')
