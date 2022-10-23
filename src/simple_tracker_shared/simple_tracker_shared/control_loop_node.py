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
from abc import abstractmethod
from .configured_node import ConfiguredNode

class ControlLoopNode(ConfiguredNode):

  def __init__(self, node_name: str):
    super().__init__(node_name)
    # setup timer and other helpers
    self.timer = self.create_timer(self.control_loop_timer_period(), self.control_loop_exe)  


  def control_loop_exe(self):

    try:
      self.control_loop()
    except RuntimeError as e:#, TypeError, NameError):
      self.get_logger().warn(f'Runtime Error {print(e)}')
      ## reload config just to be safe
      self.on_config_loaded(False)
    except cv2.error as e:
      self.get_logger().warn(f'Open CV Error {print(e)}')
      ## reload config just to be safe
      self.on_config_loaded(False)

  @abstractmethod
  def control_loop(self):
    pass

  def control_loop_timer_period(self) -> int:
    return 0.1
