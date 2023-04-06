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

import traceback as tb
import rclpy
from rclpy.qos import QoSProfile, QoSPresetProfiles
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from typing import List
from sensor_msgs.msg import Image
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .synthetic_data import DroneSyntheticData, PlaneSyntheticData
from .simulation_test import SimulationTest
from .simulation_test_case import SimulationTestCase
from .simulation_test_case_runner import SimulationTestCaseRunner

class SimulationOverlayProviderNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_overlayed_video_provider')

    # setup services, publishers and subscribers
    self.sub_camera = self.create_subscription(Image, 'sky360/simulation/input_frame', self.camera_callback, 5)
    self.pub_synthetic_frame = self.create_publisher(Image, 'sky360/simulation/output_frame', 5)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def camera_callback(self, msg_image:Image):

    if not msg_image is None:

      if self.test_case_runner.active:

        try:
          input_frame = self.br.imgmsg_to_cv2(msg_image)
          frame_synthetic = self.test_case_runner.run(input_frame)

          frame_synthetic_msg = self.br.cv2_to_imgmsg(frame_synthetic, encoding=msg_image.encoding)
          frame_synthetic_msg.header = msg_image.header

          self.pub_synthetic_frame.publish(frame_synthetic_msg)        
        except Exception as e:
          self.get_logger().error(f"Exception during frame overlay simulation. Error: {e}.")
          self.get_logger().error(tb.format_exc())
      
      else:
        self.get_logger().info(f'{self.get_name()} Overlay simulation complete.')

  def config_list(self) -> List[str]:
    return ['frame_provider_resize_dimension_h', 'frame_provider_resize_dimension_w']

  def validate_config(self) -> bool:
    valid = True
    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()

      self.test_case_runner = SimulationTestCaseRunner(
        [
          SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=3, loop=False)], (960, 960), simulation_name='Drone'),
          SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=3, loop=False)], (960, 960), simulation_name='Plane'),
          SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=3, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=3, loop=False)], (960, 960), simulation_name='Drone & Plane'),

          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=5, loop=False)], (1440, 1440), simulation_name='Drone'),
          #SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=5, loop=True)], (1440, 1440), simulation_name='Plane'),
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=5, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=5, loop=False)], (1440, 1440), simulation_name='Drone & Plane'),
          
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=8, loop=False)], (2160, 2160), simulation_name='Drone'),
          #SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=8, loop=False)], (2160, 2160), simulation_name='Plane'),
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=8, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=8, loop=False)], (2160, 2160), simulation_name='Drone & Plane'),

          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=11, loop=False)], (2880, 2880), simulation_name='Drone'),
          #SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=11, loop=False)], (2880, 2880), simulation_name='Plane'),
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=11, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=11, loop=False)], (2880, 2880), simulation_name='Drone & Plane'),

          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=15, loop=False)], (3600, 3600), simulation_name='Drone'),
          #SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=15, loop=False)], (3600, 3600), simulation_name='Plane'),       
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=15, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=15, loop=False)], (3600, 3600), simulation_name='Drone & Plane'),
        ]
      )

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = SimulationOverlayProviderNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()
