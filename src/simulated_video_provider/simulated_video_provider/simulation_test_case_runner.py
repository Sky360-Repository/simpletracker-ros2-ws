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
from simple_tracker_shared.utils import get_optimal_font_scale

class SimulationTestCaseRunner():

  def __init__(self, test_cases):
    self.test_cases = test_cases
    self.completed_test_cases = []
    self.running_test_case = None
    self.counter = 0
    self.warmup_threshold = 0
    self.completed_frame = None
    if len(self.test_cases) > 0:
      self.running_test_case = self.test_cases[0]
      self.running_test_case.notify_of_frame_change()
      self.completed_frame_dimensions = self.running_test_case.dimensions #(w,h)

  @property
  def active(self):
    return self.running_test_case is not None

  @property
  def image(self):
    return self.running_test_case.image

  def run(self):

    if not self.running_test_case is None:

      if not self.running_test_case.active:
        self.completed_test_cases.append(self.running_test_case)
        self.test_cases.remove(self.running_test_case)
        self.running_test_case = None        

      if self.running_test_case is None:
        if len(self.test_cases) > 0:
          self.running_test_case = self.test_cases[0]
          self.running_test_case.notify_of_frame_change()
          self.completed_frame_dimensions = self.running_test_case.dimensions #(w,h)
          self.counter = 0

      if not self.running_test_case is None:
        if self.counter > self.warmup_threshold:
          return self.running_test_case.run()
        else:
          self.counter += 1
          return self.running_test_case.image

    return self._completed_frame()

  def _completed_frame(self):

    if self.completed_frame is None:
      (w, h) = self.completed_frame_dimensions
      self.completed_frame = np.full((w, h, 3) , (255, 255, 255), np.uint8)      
      simulation_message = f"(Sky360) Simulation Complete"
      fontScale = get_optimal_font_scale(simulation_message, int(w-50))
      cv2.putText(self.completed_frame, simulation_message, (25, int(h/2)), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (50, 170, 50), 2)

    return self.completed_frame
