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
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from typing import List
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame, BoundingBox, BoundingBoxArray, TrackingState, Track, TrackArray, CenterPoint
from simple_tracker_shared.control_loop_node import ControlLoopNode
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from .video_tracker import VideoTracker

class TrackProviderNode(ControlLoopNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('sky360_track_provider')

    # setup services, publishers and subscribers
    self.sub_masked_frame = self.create_subscription(Frame, 'sky360/frames/masked/v1', self.frame_callback, 10)#, subscriber_qos_profile)
    self.sub_detector_bounding_boxes = self.create_subscription(BoundingBoxArray, 'sky360/detector/bgs/bounding_boxes/v1', self.bboxes_callback, 10)#, subscriber_qos_profile)
    self.pub_tracker_tracks = self.create_publisher(TrackArray, 'sky360/tracker/tracks/v1', 10)#, publisher_qos_profile)
    self.pub_tracker_tracking_state = self.create_publisher(TrackingState, 'sky360/tracker/tracking_state/v1', 10)#, get_topic_publisher_qos_profile(QoSReliabilityPolicy.BEST_EFFORT))

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def frame_callback(self, msg_frame:Frame):
    self.msg_frame = msg_frame

  def bboxes_callback(self, msg_bounding_box_array:BoundingBoxArray):
    self.msg_bounding_box_array = msg_bounding_box_array

  def control_loop(self):  

    if self.msg_frame != None:

      frame = self.br.imgmsg_to_cv2(self.msg_frame.frame)

      if self.msg_bounding_box_array != None:
        bboxes = [self._msg_to_bbox(x) for x in self.msg_bounding_box_array.boxes]
        self.video_tracker.update_trackers(bboxes, frame)

      track_array_msg = TrackArray()
      track_array_msg.epoch = self.msg_frame.epoch
      track_array_msg.fps = self.msg_frame.fps
      track_array_msg.frame_count = self.msg_frame.frame_count
      track_array_msg.tracks = [self._track_to_msg(tracker) for tracker in self.video_tracker.live_trackers]
      track_array_msg.frame = self.msg_frame.frame
      self.pub_tracker_tracks.publish(track_array_msg)
      
      tracking_msg = TrackingState()
      tracking_msg.epoch = self.msg_frame.epoch
      tracking_msg.fps = self.msg_frame.fps
      tracking_msg.frame_count = self.msg_frame.frame_count
      tracking_msg.trackable = sum(map(lambda x: x.is_tracking(), self.video_tracker.live_trackers))
      tracking_msg.alive = len(self.video_tracker.live_trackers)
      tracking_msg.started = self.video_tracker.total_trackers_started
      tracking_msg.ended = self.video_tracker.total_trackers_finished
      self.pub_tracker_tracking_state.publish(tracking_msg)


  def _msg_to_bbox(self, bbox_msg):
    return bbox_msg.x, bbox_msg.y, bbox_msg.w, bbox_msg.h

  def _track_to_msg(self, tracker):

    x, y, w, h = tracker.get_bbox()
    bbox_msg = BoundingBox()
    bbox_msg.x = x
    bbox_msg.y = y
    bbox_msg.w = w
    bbox_msg.h = h

    track_msg = Track()
    track_msg.id = tracker.id
    track_msg.state = tracker.tracking_state
    track_msg.bbox = bbox_msg

    for center_point in tracker.center_points:
      (x,y) = center_point[0]
      state = center_point[1]
      cpoint = CenterPoint()
      cpoint.x = x
      cpoint.y = y
      cpoint.state = state
      track_msg.path.append(cpoint)

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
      self.msg_frame: Frame = None
      self.msg_bounding_box_array: BoundingBoxArray = None
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