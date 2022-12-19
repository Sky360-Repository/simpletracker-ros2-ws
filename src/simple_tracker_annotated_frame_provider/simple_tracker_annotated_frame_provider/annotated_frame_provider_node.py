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
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import cv2
import numpy as np
from typing import List
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2DArray
from simple_tracker_interfaces.msg import TrackingState, TrackTrajectory, TrackTrajectoryArray
from simple_tracker_shared.control_loop_node import ConfiguredNode
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
 
class AnnotatedFrameProviderNode(ConfiguredNode):

  PROVISIONARY_TARGET = 1
  ACTIVE_TARGET = 2
  LOST_TARGET = 3

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('annotated_frame_provider')

    # setup services, publishers and subscribers
    self.pub_annotated_frame = self.create_publisher(Image, 'sky360/frames/annotated/v1', 10)#, publisher_qos_profile)

    self.sub_masked_frame = message_filters.Subscriber(self, Image, 'sky360/frames/masked/v1')#, subscriber_qos_profile)
    self.sub_tracking_state = message_filters.Subscriber(self, TrackingState, 'sky360/tracker/tracking_state/v1')#, get_topic_subscriber_qos_profile(QoSReliabilityPolicy.BEST_EFFORT))    
    self.sub_tracker_detections = message_filters.Subscriber(self, Detection2DArray, 'sky360/tracker/detections/v1')#, subscriber_qos_profile)
    self.sub_tracker_trajectories = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/trajectories/v1')#, subscriber_qos_profile)

    # setup the time synchronizer and register the subscriptions and callback
    self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_tracking_state, self.sub_tracker_detections, self.sub_tracker_trajectories], 10)
    self.time_synchronizer.registerCallback(self.synced_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def synced_callback(self, masked_frame:Image, msg_tracking_state:TrackingState, msg_detection_array:Detection2DArray, msg_trajectory_array:TrackTrajectoryArray):

    if masked_frame is not None and msg_tracking_state is not None and msg_detection_array is not None and msg_trajectory_array is not None:

      annotated_frame = self.br.imgmsg_to_cv2(masked_frame)

      status_message = f"(Sky360) Tracker Status: trackable:{msg_tracking_state.trackable}, alive:{msg_tracking_state.alive}, started:{msg_tracking_state.started}, ended:{msg_tracking_state.ended}"

      cv2.putText(annotated_frame, status_message, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_colour, self.font_thickness)

      total_height = annotated_frame.shape[:2][0]
      total_width = annotated_frame.shape[:2][1]

      cropped_track_counter = 0
      enable_cropped_tracks = self.app_configuration['visualiser_show_cropped_tracks']
      zoom_factor = self.app_configuration['visualiser_cropped_zoom_factor']  

      for detection in msg_detection_array.detections:
        (x, y, w, h) = self._get_sized_bbox(detection.bbox)        
        p1 = (int(x), int(y))
        p2 = (int(x + w), int(y + h))
        #color = self._color(track.state)
        color = self._color(self.ACTIVE_TARGET) # TODO: Fix this
        cv2.rectangle(annotated_frame, p1, p2, color, self.bbox_line_thickness, 1)
        cv2.putText(annotated_frame, detection.id, (p1[0], p1[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, color, self.font_thickness)

        if enable_cropped_tracks: #and track.state == self.ACTIVE_TARGET:  # TODO: Fix this
          margin = 0 if cropped_track_counter == 0 else 10
          zoom_w, zoom_h = w * zoom_factor, h * zoom_factor              
          cropped_image_x, cropped_image_y = (10+(cropped_track_counter*zoom_w)+margin), (total_height-(zoom_h+10))
          if cropped_image_x + zoom_w < total_width:
            try:
              annotated_frame[cropped_image_y:cropped_image_y+zoom_h,cropped_image_x:cropped_image_x+zoom_w] = cv2.resize(annotated_frame[y:y+h, x:x+w], None, fx=zoom_factor, fy=zoom_factor)
            except TypeError:
              pass
            finally:
              cropped_track_counter += 1

      for trajectory in msg_trajectory_array.trajectories:
        self._add_trajectory(trajectory, annotated_frame)

      frame_annotated_msg = self.br.cv2_to_imgmsg(annotated_frame, masked_frame.encoding)
      frame_annotated_msg.header = masked_frame.header
      self.pub_annotated_frame.publish(frame_annotated_msg)

  def config_list(self) -> List[str]:
    return ['visualiser_frame_source','visualiser_font_size', 'visualiser_font_thickness', 'visualiser_bbox_line_thickness', 'visualiser_bbox_size',
      'visualiser_show_cropped_tracks', 'visualiser_cropped_zoom_factor']

  def validate_config(self) -> bool:
    valid = True
    
    if self.app_configuration['visualiser_font_size'] == None:
      self.get_logger().error('The visualiser_font_size config entry is null')
      valid = False

    if self.app_configuration['visualiser_font_thickness'] == None:
      self.get_logger().error('The visualiser_font_thickness config entry is null')
      valid = False

    return valid          

  def on_config_loaded(self, init: bool):
    if init:
      self.font_colour = (50, 170, 50)
      self.prediction_colour = (255, 0, 0)
      self.prediction_radius = 1
      self.msg_frame: Image = None
      self.msg_tracking_state: TrackingState = None
      self.msg_track_array: TrackArray = None
      self.br = CvBridge()      

    self.font_size = self.app_configuration['visualiser_font_size']
    self.font_thickness = self.app_configuration['visualiser_font_thickness']
    self.bbox_line_thickness = self.app_configuration['visualiser_bbox_line_thickness']
    self.frame_type = self.app_configuration['visualiser_frame_source']

  def _get_sized_bbox(self, bbox_msg: BoundingBox2D):
    x, y, w, h = (int(bbox_msg.center.position.x - (bbox_msg.size_x / 2)), int(bbox_msg.center.position.y - (bbox_msg.size_y / 2)), bbox_msg.size_x, bbox_msg.size_y)
    bbox = (x, y, w, h)
    if self.app_configuration['visualiser_bbox_size'] is not None:
        size = self.app_configuration['visualiser_bbox_size']
        if w < size and h < size:
          x1 = int(x+(w/2)) - int(size/2)
          y1 = int(y+(h/2)) - int(size/2)
          bbox = (x1, y1, size, size)
    return bbox

  def _add_trajectory(self, track: TrackTrajectory, frame):
    trajectory_array = track.trajectory
    p_point_ = None
    for t_point in trajectory_array:
      if not p_point_ is None:
        cv2.line(frame, 
          (int(p_point_.center.x), int(p_point_.center.y)), (int(t_point.center.x), int(t_point.center.y)), 
          color = self.prediction_colour if t_point.prediction else self._color(t_point.state), 
          thickness = self.bbox_line_thickness)

      p_point_ = t_point

  def _is_point_contained_in_bbox(self, bbox, point):
    x, y, w, h = bbox
    x0, y0 = point
    return x <= x0 < x + w and y <= y0 < y + h

  def _color(self, tracking_state):
    return {
            self.PROVISIONARY_TARGET: (25, 175, 175),
            self.ACTIVE_TARGET: (50, 170, 50),
            self.LOST_TARGET: (50, 50, 225)
        }[tracking_state]


def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = AnnotatedFrameProviderNode(subscriber_qos_profile, publisher_qos_profile)

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