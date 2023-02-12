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
import message_filters
from rclpy.qos import QoSProfile, QoSPresetProfiles, qos_profile_sensor_data
from typing import List
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from simple_tracker_interfaces.msg import TrackingState, TrackTrajectoryArray
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .annotated_frame_creator import AnnotatedFrameCreator

class AnnotatedFrameProviderNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('annotated_frame_provider')

    # setup services, publishers and subscribers
    self.pub_annotated_frame = self.create_publisher(Image, 'sky360/frames/annotated', publisher_qos_profile)

    self.sub_masked_frame = message_filters.Subscriber(self, Image, 'sky360/frames/masked', qos_profile=subscriber_qos_profile)
    self.sub_tracking_state = message_filters.Subscriber(self, TrackingState, 'sky360/tracker/tracking_state', qos_profile=subscriber_qos_profile)
    self.sub_tracker_detections = message_filters.Subscriber(self, Detection2DArray, 'sky360/tracker/detections', qos_profile=subscriber_qos_profile)
    self.sub_tracker_trajectory = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/trajectory', qos_profile=subscriber_qos_profile)
    self.sub_tracker_prediction = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/prediction', qos_profile=subscriber_qos_profile)

    # setup the time synchronizer and register the subscriptions and callback
    self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_tracking_state, 
      self.sub_tracker_detections, self.sub_tracker_trajectory, self.sub_tracker_prediction], 10)
    self.time_synchronizer.registerCallback(self.synced_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def synced_callback(self, msg_masked_frame:Image, msg_tracking_state:TrackingState, msg_detection_array:Detection2DArray, 
    msg_trajectory_array:TrackTrajectoryArray, msg_prediction_array:TrackTrajectoryArray):

    if msg_masked_frame is not None and msg_tracking_state is not None and msg_detection_array is not None and msg_trajectory_array is not None:

      try:
        annotated_frame = self.creator.create(self.br.imgmsg_to_cv2(msg_masked_frame), msg_tracking_state, msg_detection_array, msg_trajectory_array, msg_prediction_array)
        frame_annotated_msg = self.br.cv2_to_imgmsg(annotated_frame, msg_masked_frame.encoding)
        frame_annotated_msg.header = msg_masked_frame.header
        self.pub_annotated_frame.publish(frame_annotated_msg)
      except Exception as e:
        self.get_logger().error(f"Exception during the annotated frame provider. Error: {e}.")
        self.get_logger().error(tb.format_exc())

  def config_list(self) -> List[str]:
    return ['visualiser_frame_source', 'visualiser_bbox_line_thickness', 'visualiser_bbox_size', 'visualiser_show_cropped_tracks', 'visualiser_cropped_zoom_factor',
    'frame_provider_resize_dimension_h', 'frame_provider_resize_dimension_w']

  def validate_config(self) -> bool:
    valid = True
    return valid          

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()      

    self.creator = AnnotatedFrameCreator(self.app_configuration)

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = qos_profile_sensor_data #get_topic_subscriber_qos_profile()
  publisher_qos_profile = qos_profile_sensor_data #get_topic_publisher_qos_profile()

  node = AnnotatedFrameProviderNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()

if __name__ == '__main__':
  main()