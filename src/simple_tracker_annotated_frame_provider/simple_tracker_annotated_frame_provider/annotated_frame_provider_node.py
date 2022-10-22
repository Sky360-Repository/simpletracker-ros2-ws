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
import cv2
from typing import List
from simple_tracker_interfaces.msg import Frame
from simple_tracker_interfaces.msg import TrackingState
from simple_tracker_interfaces.msg import TrackArray
from simple_tracker_interfaces.msg import Track
from simple_tracker_interfaces.msg import BoundingBox
from simple_tracker_shared.control_loop_node import ControlLoopNode
from cv_bridge import CvBridge
 
class AnnotatedFrameProviderNode(ControlLoopNode):

  PROVISIONARY_TARGET = 1
  ACTIVE_TARGET = 2
  LOST_TARGET = 3

  def __init__(self):
    super().__init__('annotated_frame_provider')

    # setup services, publishers and subscribers
    self.pub_annotated_frame = self.create_publisher(Frame, 'sky360/frames/annotated/v1', 10)
    self.tracking_state_sub = self.create_subscription(TrackingState, 'sky360/tracker/tracking_state/v1', self.tracking_state_callback, 10)
    self.tracker_tracks_sub = self.create_subscription(TrackArray, 'sky360/tracker/tracks/v1', self.tracks_callback, 10)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def fp_original_callback(self, msg_frame:Frame):
    self.msg_frame = msg_frame

  def tracking_state_callback(self, msg_tracking_state:TrackingState):
    self.msg_tracking_state = msg_tracking_state

  def tracks_callback(self, msg_track_array:TrackArray):
    self.msg_track_array = msg_track_array

  def control_loop(self):

    if self.msg_track_array != None:

      annotated_frame = self.br.imgmsg_to_cv2(self.msg_track_array.frame)

      if self.msg_tracking_state != None and self.msg_track_array != None:

        status_message = f"(Sky360) Tracker Status: trackable:{self.msg_tracking_state.trackable}, alive:{self.msg_tracking_state.alive}, started:{self.msg_tracking_state.started}, ended:{self.msg_tracking_state.ended}"
        frame_message = f"(Sky360) count:{self.msg_tracking_state.frame_count}, epoch:{self.msg_tracking_state.epoch}, fps:{self.msg_tracking_state.fps}"

        cv2.putText(annotated_frame, status_message, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_colour, self.font_thickness)
        cv2.putText(annotated_frame, frame_message, (25, 50), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_colour, self.font_thickness)

        for track in self.msg_track_array.tracks:
          (x, y, w, h) = self._get_sized_bbox(track.bbox)
          p1 = (int(x), int(y))
          p2 = (int(x + w), int(y + h))
          color = self._color(track.state)
          cv2.rectangle(annotated_frame, p1, p2, color, self.bbox_line_thickness, 1)
          cv2.putText(annotated_frame, str(track.id), (p1[0], p1[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, color, self.font_thickness)
          self._add_tracked_path(track, annotated_frame)
          self._add_predicted_point(track, annotated_frame)

      frame_annotated_msg = Frame()
      frame_annotated_msg.epoch = self.msg_track_array.epoch
      frame_annotated_msg.fps = self.msg_track_array.fps
      frame_annotated_msg.frame_count = self.msg_track_array.frame_count
      frame_annotated_msg.frame = self.br.cv2_to_imgmsg(annotated_frame)

      self.pub_annotated_frame.publish(frame_annotated_msg)

  def config_list(self) -> List[str]:
    return ['visualiser_frame_source','visualiser_font_size', 'visualiser_font_thickness', 'visualiser_bbox_line_thickness', 'visualiser_bbox_size']

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
      self.msg_frame: Frame = None
      self.msg_tracking_state: TrackingState = None
      self.msg_track_array: TrackArray = None
      self.br = CvBridge()      

    self.font_size = self.app_configuration['visualiser_font_size']
    self.font_thickness = self.app_configuration['visualiser_font_thickness']
    self.bbox_line_thickness = self.app_configuration['visualiser_bbox_line_thickness']
    self.frame_type = self.app_configuration['visualiser_frame_source']

  def _get_sized_bbox(self, bbox_msg: BoundingBox):
    x = bbox_msg.x
    y = bbox_msg.y
    w = bbox_msg.w
    h = bbox_msg.h
    bbox = (x, y, w, h)
    if self.app_configuration['visualiser_bbox_size'] is not None:
        size = self.app_configuration['visualiser_bbox_size']
        if w < size and h < size:
          x1 = int(x+(w/2)) - int(size/2)
          y1 = int(y+(h/2)) - int(size/2)
          bbox = (x1, y1, size, size)
    return bbox

  def _add_tracked_path(self, track: Track, frame):
    bbox = self._get_sized_bbox(track.bbox)
    path_points = track.path
    previous_point = None
    for path_point in path_points:
        if not previous_point is None:
            if not self._is_point_contained_in_bbox(bbox, (path_point.x, path_point.y)):
                cv2.line(frame, (previous_point.x, previous_point.y), (path_point.x, path_point.y), self._color(path_point.state), thickness=self.bbox_line_thickness)
        previous_point = path_point

  def _add_predicted_point(self, track: Track, frame):
    predicted_center_point = track.predicted_point
    cv2.circle(frame, (predicted_center_point.x, predicted_center_point.y), radius=self.prediction_radius, color=self.prediction_colour, thickness=self.bbox_line_thickness)

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
  annotated_frame_provider_node = AnnotatedFrameProviderNode()
  rclpy.spin(annotated_frame_provider_node)
  annotated_frame_provider_node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()