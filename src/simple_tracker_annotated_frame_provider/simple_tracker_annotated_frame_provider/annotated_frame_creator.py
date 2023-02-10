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
import rclpy.logging
import cv2
from vision_msgs.msg import BoundingBox2D, Detection2DArray
from simple_tracker_interfaces.msg import TrackingState, TrackTrajectoryArray
from simple_tracker_shared.utils import get_optimal_font_scale
from simple_tracker_shared.enumerations import TrackingStateEnum

class AnnotatedFrameCreator():

  def __init__(self, settings):
    self.settings = settings
    self.logger = rclpy.logging.get_logger('annotated_frame_creator')
    self.font_colour = (50, 170, 50)
    self.prediction_colour = (255, 0, 0)
    self.prediction_radius = 1
    self.bbox_line_thickness = self.settings['visualiser_bbox_line_thickness']
    self.frame_type = self.settings['visualiser_frame_source']
    self.fontScale = get_optimal_font_scale("(Sky360) Tracker Status: trackable:0, alive:0, started:0, ended:0", 
      int(self.settings['frame_provider_resize_dimension_w']*0.65))    

  def create(self, annotated_frame, msg_tracking_state:TrackingState, msg_detection_array:Detection2DArray, 
    msg_trajectory_array:TrackTrajectoryArray, msg_prediction_array:TrackTrajectoryArray):

    cropped_track_counter = 0
    enable_cropped_tracks = self.settings['visualiser_show_cropped_tracks']
    zoom_factor = self.settings['visualiser_cropped_zoom_factor']
    detections = {}
    final_trajectory_points = {}

    if enable_cropped_tracks:
      annotated_frame_clone = annotated_frame.copy()

    total_height = annotated_frame.shape[:2][0]
    total_width = annotated_frame.shape[:2][1]

    status_message = f"(Sky360) Tracker Status: trackable:{msg_tracking_state.trackable}, alive:{msg_tracking_state.alive}, started:{msg_tracking_state.started}, ended:{msg_tracking_state.ended}"
    cv2.putText(annotated_frame, status_message, (25, 50), cv2.FONT_HERSHEY_SIMPLEX, self.fontScale, self.font_colour, 2)

    for detection in msg_detection_array.detections:

      id_arr = detection.id.split("-")

      id = id_arr[0]
      tracking_state = TrackingStateEnum(int(id_arr[1]))

      (x, y, w, h) = self._get_sized_bbox(detection.bbox)
      detections[detection.id] = (x, y, w, h)
      p1 = (int(x), int(y))
      p2 = (int(x + w), int(y + h))
      color = self._color(tracking_state)
      cv2.rectangle(annotated_frame, p1, p2, color, self.bbox_line_thickness, 1)
      cv2.putText(annotated_frame, id, (p1[0], p1[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, self.fontScale, color, 2)

      if enable_cropped_tracks and tracking_state == TrackingStateEnum.ActiveTarget:
        margin = 0 if cropped_track_counter == 0 else 10
        zoom_w, zoom_h = w * zoom_factor, h * zoom_factor              
        cropped_image_x, cropped_image_y = (10+(cropped_track_counter*zoom_w)+margin), (total_height-(zoom_h+10))
        if cropped_image_x + zoom_w < total_width:
          try:
            annotated_frame[cropped_image_y:cropped_image_y+zoom_h,cropped_image_x:cropped_image_x+zoom_w] = cv2.resize(annotated_frame_clone[y:y+h, x:x+w], None, fx=zoom_factor, fy=zoom_factor)
          except TypeError:
            pass
          finally:
            cropped_track_counter += 1

    for trajectory in msg_trajectory_array.trajectories:
      trajectory_array = trajectory.trajectory
      previous_trajectory_point = None
      for trajectory_point in trajectory_array:
        if not previous_trajectory_point is None:
          #if not self._is_point_contained_in_bbox(detections[trajectory.id], (trajectory_point.center.x, trajectory_point.center.y)):
            cv2.line(annotated_frame, 
              (int(previous_trajectory_point.center.x), int(previous_trajectory_point.center.y)), (int(trajectory_point.center.x), int(trajectory_point.center.y)), 
              color = self._color(TrackingStateEnum(trajectory_point.tracking_state)), thickness = self.bbox_line_thickness)
        previous_trajectory_point = trajectory_point
        final_trajectory_points[trajectory.id] = previous_trajectory_point

    for prediction in msg_prediction_array.trajectories:
      prediction_array = prediction.trajectory
      if prediction.id in final_trajectory_points:
        previous_prediction_point = final_trajectory_points[prediction.id]
        for prediction_point in prediction_array:
          if not previous_prediction_point is None:
            #if not self._is_point_contained_in_bbox(detections[trajectory.id], (prediction_point.center.x, prediction_point.center.y)):
              cv2.line(annotated_frame, 
                (int(previous_prediction_point.center.x), int(previous_prediction_point.center.y)), (int(prediction_point.center.x), int(prediction_point.center.y)), 
                color = self.prediction_colour, thickness = self.bbox_line_thickness)

          previous_prediction_point = prediction_point

    return annotated_frame

  def _get_sized_bbox(self, bbox_msg: BoundingBox2D):
    x, y, w, h = (int(bbox_msg.center.position.x - (bbox_msg.size_x / 2)), int(bbox_msg.center.position.y - (bbox_msg.size_y / 2)), bbox_msg.size_x, bbox_msg.size_y)
    bbox = (x, y, w, h)
    if self.settings['visualiser_bbox_size'] is not None:
        size = self.settings['visualiser_bbox_size']
        if w < size and h < size:
          x1 = int(x+(w/2)) - int(size/2)
          y1 = int(y+(h/2)) - int(size/2)
          bbox = (x1, y1, size, size)
    return bbox

  def _is_point_contained_in_bbox(self, bbox, point):
    x, y, w, h = bbox
    x0, y0 = point
    return x <= x0 < x + w and y <= y0 < y + h

  def _color(self, tracking_state: TrackingStateEnum):
    return {
            TrackingStateEnum.ProvisionaryTarget: (25, 175, 175),
            TrackingStateEnum.ActiveTarget: (50, 170, 50),
            TrackingStateEnum.LostTarget: (50, 50, 225)
        }[tracking_state]
