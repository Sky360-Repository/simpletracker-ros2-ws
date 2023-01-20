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

class SimulationRunner():

  def __init__(self, test_cases):
    self.test_cases = test_cases
    self.completed_test_cases = []
    self.running_test_case = None
    if len(self.test_cases) > 0:
        self.running_test_case = self.test_cases[0]

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

      if len(self.test_cases) > 0:
        self.running_test_case = self.test_cases[0]

      if not self.running_test_case is None:
        return self.running_test_case.run()

    return None

