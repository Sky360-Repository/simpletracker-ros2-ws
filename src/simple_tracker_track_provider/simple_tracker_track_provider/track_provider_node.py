import datetime
import rclpy
import cv2
from typing import List
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame
from simple_tracker_interfaces.msg import BoundingBox
from simple_tracker_interfaces.msg import BoundingBoxArray
from simple_tracker_interfaces.msg import TrackingState
from simple_tracker_interfaces.msg import Track
from simple_tracker_interfaces.msg import TrackArray
from simple_tracker_interfaces.msg import CenterPoint
from simple_tracker_shared.configured_node import ConfiguredNode
from .video_tracker import VideoTracker

class TrackProviderNode(ConfiguredNode):

  def __init__(self):
    super().__init__('sky360_track_provider')

    # setup services, publishers and subscribers
    self.sub_original_frame = self.create_subscription(Frame, 'sky360/frames/original/v1', self.frame_callback, 10)
    self.sub_detector_bounding_boxes = self.create_subscription(BoundingBoxArray, 'sky360/detector/bounding_boxes/v1', self.bboxes_callback, 10)
    self.pub_tracker_tracks = self.create_publisher(TrackArray, 'sky360/tracker/tracks/v1', 10)
    self.pub_tracker_tracking_state = self.create_publisher(TrackingState, 'sky360/tracker/tracking_state/v1', 10)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def frame_callback(self, data:Frame):

    self.frame = self.br.imgmsg_to_cv2(data.frame)

    track_array_msg = TrackArray()
    track_array_msg.epoch = data.epoch
    track_array_msg.fps = data.fps
    track_array_msg.frame_count = data.frame_count
    track_array_msg.tracks = [self._track_to_msg(tracker) for tracker in self.video_tracker.live_trackers]
    self.pub_tracker_tracks.publish(track_array_msg)
    
    tracking_msg = TrackingState()
    tracking_msg.epoch = data.epoch
    tracking_msg.fps = data.fps
    tracking_msg.frame_count = data.frame_count
    tracking_msg.trackable = sum(map(lambda x: x.is_tracking(), self.video_tracker.live_trackers))
    tracking_msg.alive = len(self.video_tracker.live_trackers)
    tracking_msg.started = self.video_tracker.total_trackers_started
    tracking_msg.ended = self.video_tracker.total_trackers_finished
    self.pub_tracker_tracking_state.publish(tracking_msg)

  def bboxes_callback(self, data:BoundingBoxArray):

    if not self.configuration_loaded or self.frame is None:
        pass

    bboxes = [self._msg_to_bbox(x) for x in data.boxes]
    self.video_tracker.update_trackers(bboxes, self.frame)

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
      'track_orphaned_threshold']

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
  track_provider_node = TrackProviderNode()
  rclpy.spin(track_provider_node)
  track_provider_node.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()