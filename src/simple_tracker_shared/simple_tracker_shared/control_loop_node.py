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
