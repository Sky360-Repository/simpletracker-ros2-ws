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

from abc import abstractmethod
#from concurrent.futures import ThreadPoolExecutor

from .configured_node import ConfiguredNode

class ControlLoopNode(ConfiguredNode):

  def __init__(self, node_name: str):
    super().__init__(node_name)
    #self.myExecutor = ThreadPoolExecutor(max_workers=4)
    # setup timer and other helpers
    self.timer = self.create_timer(self.control_loop_timer_period(), self.control_loop)  

  #@abstractmethod
  #def run(self):
  #  pass

  @abstractmethod
  def control_loop(self):
    pass

  #def start(self):
  #  self.myExecutor.submit(self.run())

  def control_loop_timer_period(self) -> int:
    return 0.1
