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

import datetime
import rclpy
import message_filters
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from typing import List
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray, Detection2D, Detection2DArray
from simple_tracker_interfaces.msg import TrackingState, TrackPoint, TrackTrajectory, TrackTrajectoryArray
from simple_tracker_shared.control_loop_node import ConfiguredNode
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .video_tracker import VideoTracker

class TrackProviderNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_track_provider')

    # setup services, publishers and subscribers
    self.sub_masked_frame = message_filters.Subscriber(self, Image, 'sky360/frames/masked/v1')#, subscriber_qos_profile)
    self.sub_detector_bounding_boxes = message_filters.Subscriber(self, BoundingBox2DArray, 'sky360/detector/bgs/bounding_boxes/v1')#, subscriber_qos_profile)
    
    self.pub_tracker_tracking_state = self.create_publisher(TrackingState, 'sky360/tracker/tracking_state/v1', 10)#, get_topic_publisher_qos_profile(QoSReliabilityPolicy.BEST_EFFORT))
    self.pub_tracker_detects = self.create_publisher(Detection2DArray, 'sky360/tracker/detections/v1', 10)#, publisher_qos_profile)
    self.pub_tracker_trajectories = self.create_publisher(TrackTrajectoryArray, 'sky360/tracker/trajectories/v1', 10)#, publisher_qos_profile)

    # setup the time synchronizer and register the subscriptions and callback
    self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_detector_bounding_boxes], 10)
    self.time_synchronizer.registerCallback(self.synced_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def synced_callback(self, msg_frame:Image, msg_bounding_box_array:BoundingBox2DArray):

    if msg_frame is not None and msg_bounding_box_array is not None:

      frame = self.br.imgmsg_to_cv2(msg_frame)

      bboxes = [self._msg_to_bbox(x) for x in msg_bounding_box_array.boxes]

      self.video_tracker.update_trackers(bboxes, frame)

      detect_array_msg = Detection2DArray()
      detect_array_msg.header = msg_frame.header
      detect_array_msg.detections = [self._detects_to_msg(tracker) for tracker in self.video_tracker.live_trackers]

      trajectory_array_msg = TrackTrajectoryArray()
      trajectory_array_msg.header = msg_frame.header
      trajectory_array_msg.trajectories = [self._trajectories_to_msg(tracker) for tracker in self.video_tracker.live_trackers]

      tracking_msg = TrackingState()
      tracking_msg.header = msg_frame.header
      tracking_msg.trackable = sum(map(lambda x: x.is_tracking(), self.video_tracker.live_trackers))
      tracking_msg.alive = len(self.video_tracker.live_trackers)
      tracking_msg.started = self.video_tracker.total_trackers_started
      tracking_msg.ended = self.video_tracker.total_trackers_finished

      self.pub_tracker_detects.publish(detect_array_msg)
      self.pub_tracker_trajectories.publish(trajectory_array_msg)
      self.pub_tracker_tracking_state.publish(tracking_msg)

  def _msg_to_bbox(self, bbox_msg: BoundingBox2D):
    x, y, w, h = (int(bbox_msg.center.position.x - (bbox_msg.size_x / 2)), int(bbox_msg.center.position.y - (bbox_msg.size_y / 2)), int(bbox_msg.size_x), int(bbox_msg.size_y))
    return x, y, w, h

  def _detects_to_msg(self, tracker):

    x, y, w, h = tracker.get_bbox()
    bbox_msg = BoundingBox2D()
    bbox_msg.center.position.x = float(x+(w/2))
    bbox_msg.center.position.y = float(y+(h/2))
    bbox_msg.center.theta = 0.0
    bbox_msg.size_x = float(w)
    bbox_msg.size_y = float(h)

    detect_msg = Detection2D()
    detect_msg.id = str(tracker.id)
    detect_msg.bbox = bbox_msg

    return detect_msg

  def _trajectories_to_msg(self, tracker):

    track_msg = TrackTrajectory()
    track_msg.id = tracker.id

    for center_point in tracker.center_points:
      (x,y) = center_point[0]
      state = center_point[1]      
      point = TrackPoint()
      point.center.x = float(x)
      point.center.y = float(y)
      point.state = state
      point.prediction = False
      track_msg.trajectory.append(point)

    for center_point in tracker.predictor_center_points:
      (x,y) = center_point
      point = TrackPoint()
      point.center.x = float(x)
      point.center.y = float(y)
      point.prediction = True
      track_msg.trajectory.append(point)

    return track_msg

  def config_list(self) -> List[str]:
    return ['tracker_type', 'tracker_detection_sensitivity', 'tracker_active_only', 'tracker_max_active_trackers', 'frame_provider_resize_dimension_h', 
      'frame_provider_resize_dimension_w', 'track_path_plotting_enabled', 'track_prediction_enabled', 'track_validation_enable', 'track_stationary_threshold', 
      'track_orphaned_threshold', 'tracker_min_centre_point_distance_between_bboxes']

  def validate_config(self) -> bool:
    valid = True
    # TODO: This needs to be expanded upon
    if self.app_configuration['tracker_detection_sensitivity'] == None:
      self.get_logger().error('The tracker_detection_sensitivity config entry is null')
      valid = False
      
    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()

    self.video_tracker = VideoTracker(self.app_configuration)


def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = TrackProviderNode(subscriber_qos_profile, publisher_qos_profile)

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