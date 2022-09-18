import datetime
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import Frame
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from simple_tracker_interfaces.msg import BoundingBox
from simple_tracker_interfaces.msg import BoundingBoxArray
from simple_tracker_interfaces.msg import TrackingState
from simple_tracker_interfaces.msg import Track
from simple_tracker_interfaces.msg import TrackArray
from .config_entry_convertor import ConfigEntryConvertor
from .configurations_client_async import ConfigurationsClientAsync
from .video_tracker import VideoTracker

class TrackProviderNode(Node):

  def __init__(self):

    super().__init__('sky360_track_provider')  

    self.configuration_list = ['tracker_type', 'tracker_detection_sensitivity', 'tracker_active_only', 'tracker_max_active_trackers',
      'track_plotting_enabled', 'track_prediction_enabled', 'track_validation_enable', 'track_stationary_threshold', 'track_orphaned_threshold']
    self.app_configuration = {}
    self.configuration_loaded = False

    # setup services, publishers and subscribers
    self.configuration_svc = ConfigurationsClientAsync()
    self.sub_masked_background_frame = self.create_subscription(Frame, 'sky360/frames/original/v1', self.frame_callback, 10)
    self.sub_masked_background_frame = self.create_subscription(BoundingBoxArray, 'sky360/detector/bounding_boxes/v1', self.bboxes_callback, 10)
    self.sub_config_updated = self.create_subscription(ConfigEntryUpdatedArray, 'sky360/config/updated/v1', self.config_updated_callback, 10)
    self.pub_tracker_tracks = self.create_publisher(TrackArray, 'sky360/tracker/tracks/v1', 10)
    self.pub_tracker_tracking_state = self.create_publisher(TrackingState, 'sky360/tracker/tracking_state/v1', 10)

    # setup timer and other helpers
    self.br = CvBridge()
    
    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def frame_callback(self, data:Frame):

    # TODO: This configuration update thing needs to happen in the background
    if not self.configuration_loaded:
      self._load_and_validate_config()
      self.video_tracker = VideoTracker(self.app_configuration)
      self.configuration_loaded = True

    self.frame = self.br.imgmsg_to_cv2(data.frame)

    track_array_msg = TrackArray()
    track_array_msg.frame_count = data.frame_count
    track_array_msg.tracks = [self._track_to_msg(tracker) for tracker in self.video_tracker.live_trackers]
    self.pub_tracker_tracks.publish(track_array_msg)
    
    tracking_msg = TrackingState()
    tracking_msg.frame_count = data.frame_count
    tracking_msg.trackable = sum(map(lambda x: x.is_tracking(), self.video_tracker.live_trackers))
    tracking_msg.alive = len(self.video_tracker.live_trackers)
    tracking_msg.started = self.video_tracker.total_trackers_started
    tracking_msg.ended = self.video_tracker.total_trackers_finished
    self.pub_tracker_tracking_state.publish(tracking_msg)

  def bboxes_callback(self, data:BoundingBoxArray):

    # TODO: This configuration update thing needs to happen in the background
    # TODO: This whole thing might be better as a service
    if not self.configuration_loaded or self.frame is None:
        return

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

    return track_msg

  def config_updated_callback(self, msg:ConfigEntryUpdatedArray):

    for key in msg.keys:
      if key in self.app_configuration.keys():
        self.configuration_loaded = False
        self.get_logger().info('Receiving updated configuration notification, reload')
        break

  def _load_and_validate_config(self):

      if not self.configuration_loaded:
        self._load_config()
        
        # TODO: What is the best way of exiting out of a launch script when the configuration validation fails
        valid = self._validate_config()
        if valid == False:
          self.get_logger().error('Detector Node configuration is invalid')

        self.configuration_loaded = True

  def _load_config(self):
    #self.get_logger().info(f'Loading configuration list.')

    response = self.configuration_svc.send_request(self.configuration_list)
    for config_item in response.entries:
      self.app_configuration[config_item.key] = ConfigEntryConvertor.Convert(config_item.type, config_item.value)

  def _validate_config(self):
    #self.get_logger().info(f'Validating configuration.')

    valid = True

    if self.app_configuration['tracker_detection_sensitivity'] == None:
      self.get_logger().error('The tracker_detection_sensitivity config entry is null')
      valid = False
      
    return valid

def main(args=None):

  rclpy.init(args=args)
  track_provider_node = TrackProviderNode()
  rclpy.spin(track_provider_node)
  track_provider_node.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()