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

import rclpy
import message_filters
from datetime import datetime
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py
from typing import List
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2DArray
from simple_tracker_interfaces.msg import TrackingState, TrackTrajectoryArray
from simple_tracker_shared.control_loop_node import ConfiguredNode
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile

class RosbagRecorderNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('rosbag_recorder')

    self.sub_masked_frame = message_filters.Subscriber(self, Image, 'sky360/frames/masked')#, subscriber_qos_profile)
    self.sub_tracking_state = message_filters.Subscriber(self, TrackingState, 'sky360/tracker/tracking_state')#, get_topic_subscriber_qos_profile(QoSReliabilityPolicy.BEST_EFFORT))    
    self.sub_tracker_detections = message_filters.Subscriber(self, Detection2DArray, 'sky360/tracker/detections')#, subscriber_qos_profile)
    self.sub_tracker_trajectory = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/trajectory')#, subscriber_qos_profile)
    self.sub_tracker_prediction = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/prediction')#, subscriber_qos_profile)

    # setup the time synchronizer and register the subscriptions and callback
    self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_tracking_state, 
    self.sub_tracker_detections, self.sub_tracker_trajectory, self.sub_tracker_prediction], 10)
    self.time_synchronizer.registerCallback(self.synced_callback)

    self.setup_rosbag()

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def synced_callback(self, masked_frame:Image, msg_tracking_state:TrackingState, msg_detection_array:Detection2DArray, 
    msg_trajectory_array:TrackTrajectoryArray, msg_prediction_array:TrackTrajectoryArray):

    if masked_frame is not None and msg_tracking_state is not None and msg_detection_array is not None and msg_trajectory_array is not None:
      
      if msg_tracking_state.trackable > 0:
        ns = self.get_clock().now().nanoseconds
        self.writer.write('sky360/frames/masked', serialize_message(masked_frame), ns)
        self.writer.write('sky360/tracker/tracking_state', serialize_message(msg_tracking_state), ns)
        self.writer.write('sky360/tracker/detections', serialize_message(msg_detection_array), ns)
        self.writer.write('sky360/tracker/trajectory', serialize_message(msg_trajectory_array), ns)
        self.writer.write('sky360/tracker/prediction', serialize_message(msg_prediction_array), ns)

        status_message = f"(Sky360) Tracker Status: trackable:{msg_tracking_state.trackable}, alive:{msg_tracking_state.alive}, started:{msg_tracking_state.started}, ended:{msg_tracking_state.ended}"
        self.get_logger().info(f'{self.get_name()} - {status_message}.')

  def setup_rosbag(self):

    # setup the rosbag so that we can record into it
    db_uri = f'sky360_recordings/{datetime.now().strftime("%Y_%m_%d-%H_%M_%S")}'
    self.get_logger().info(f'Creating database {db_uri}.')

    self.writer = rosbag2_py.SequentialWriter()
    
    storage_options = rosbag2_py._storage.StorageOptions(uri=db_uri, storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    self.writer.open(storage_options, converter_options)

    image_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/frames/masked', type='sensor_msgs/msg/Image', serialization_format='cdr')
    tracking_state_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/tracker/tracking_state', type='simple_tracker_interfaces/msg/TrackingState', serialization_format='cdr')
    detection_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/tracker/detections', type='vision_msgs/msg/Detection2DArray', serialization_format='cdr')
    trajectory_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/tracker/trajectory', type='simple_tracker_interfaces/msg/TrackTrajectoryArray', serialization_format='cdr')
    prediction_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/tracker/prediction', type='simple_tracker_interfaces/msg/TrackTrajectoryArray', serialization_format='cdr')

    self.writer.create_topic(image_topic_info)
    self.writer.create_topic(detection_topic_info)
    self.writer.create_topic(tracking_state_topic_info)
    self.writer.create_topic(trajectory_topic_info)
    self.writer.create_topic(prediction_topic_info)

  def config_list(self) -> List[str]:
    return []

  def validate_config(self) -> bool:
    valid = True
    return valid          

  def on_config_loaded(self, init: bool):
    if init:
      self.msg_frame: Image = None
      self.msg_tracking_state: TrackingState = None
      self.br = CvBridge()

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = RosbagRecorderNode(subscriber_qos_profile, publisher_qos_profile)

  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
      pass
  finally:
      rclpy.try_shutdown()
      node.destroy_node()
      #rclpy.rosshutdown()


if __name__ == '__main__':
  main()