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
from ament_index_python.packages import get_package_share_directory
from simple_tracker_shared.utils import frame_resize

class SimulationTestCase():

  def __init__(self, settings, logger, tests, dimension, file_name='frame_1.jpg'):
    self.settings = settings    
    self.logger = logger
    self.tests = tests
    self.dimension = dimension
    self.file_name = file_name
    self.running = True
    self.h = 0
    self.w = 0

    frames_folder = self.videos_folder = os.path.join(get_package_share_directory('simulated_video_provider'), 'still_frames')
    frame_file_path = os.path.join(frames_folder, self.file_name)

    if os.path.exists(frame_file_path):
      (self.w, self.h) = dimension
      self.frame = cv2.imread(frame_file_path, cv2.IMREAD_COLOR)
      self.frame = frame_resize(self.frame, width=self.w, height=self.h)
    else:
      self.logger.error(f'Still frame path {frame_file_path} does not exist.')      
      self.running = False

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

      status_message = f"(Sky360) Simulation frame dimensions h:{self.h}, w:{self.w}"
      cv2.putText(frame_synthetic, status_message, (25, 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 170, 50), 2)

      for test in self.tests:
        if test.running:
          test.run(frame_synthetic)
          running = True
    
      self.running = running
      if self.running:
        return frame_synthetic

    return self.frame
