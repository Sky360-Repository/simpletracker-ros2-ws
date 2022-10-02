import rclpy
import cv2
from rclpy.node import Node
from simple_tracker_interfaces.msg import Frame
from simple_tracker_interfaces.msg import TrackingState
from simple_tracker_interfaces.msg import TrackArray
from simple_tracker_interfaces.msg import Track
from simple_tracker_interfaces.msg import BoundingBox
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from simple_tracker_shared.config_entry_convertor import ConfigEntryConvertor
from simple_tracker_shared.configurations_client_async import ConfigurationsClientAsync
from cv_bridge import CvBridge
 
class AnnotatedFrameProviderNode(Node):

  PROVISIONARY_TARGET = 1
  ACTIVE_TARGET = 2
  LOST_TARGET = 3

  def __init__(self):
    super().__init__('annotated_frame_provider_node')

    self.configuration_list = ['visualiser_frame_source','visualiser_font_size', 'visualiser_font_thickness', 'visualiser_bbox_line_thickness', 'visualiser_bbox_size']
    self.app_configuration = {}
    self.configuration_loaded = False

    #self.frame_buffer = {}
    self.font_colour = (50, 170, 50)
    self.font_size = 0.5
    self.font_thickness = 1
    self.bbox_line_thickness = 1
    self.prediction_colour = (255, 0, 0)
    self.prediction_radius = 1
    self.frame_message = ''
    self.status_message = ''
    self.frame_type = 'original'

    self.configuration_svc = ConfigurationsClientAsync()
    self.sub_config_updated = self.create_subscription(ConfigEntryUpdatedArray, 'sky360/config/updated/v1', self.config_updated_callback, 10)

    # TODO: This configuration update thing needs to happen in the background
    if not self.configuration_loaded:
      self._load_and_validate_config()
      self.configuration_loaded = True

    self.pub_annotated_frame = self.create_publisher(Frame, 'sky360/frames/annotated/v1', 10)
    self.fp_original_sub = self.create_subscription(Frame, f'sky360/frames/{self.frame_type}/v1', self.fp_original_callback, 10)
    self.tracking_state_sub = self.create_subscription(TrackingState, 'sky360/tracker/tracking_state/v1', self.tracking_state_callback, 10)
    self.tracker_tracks_sub = self.create_subscription(TrackArray, 'sky360/tracker/tracks/v1', self.tracks_callback, 10)

    self.br = CvBridge()
    self.annotated_frame = None
    self.annotated_frame_count = 0

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def fp_original_callback(self, data:Frame):
    self.annotated_frame = self.br.imgmsg_to_cv2(data.frame)
    self.annotated_frame_count = data.frame_count

  def tracking_state_callback(self, data:TrackingState):
    self.status_message = f"(Sky360) Tracker Status: trackable:{data.trackable}, alive:{data.alive}, started:{data.started}, ended:{data.ended}"
    self.frame_message = f"(Sky360) {self.frame_type} frame, count:{data.frame_count}, epoch:{data.epoch}, fps:{data.fps}"

  def tracks_callback(self, data:TrackArray):

    if self.configuration_loaded == False:
      pass

    if self.annotated_frame is None:
      pass

    for track in data.tracks:
      (x, y, w, h) = self._get_sized_bbox(track.bbox)
      p1 = (int(x), int(y))
      p2 = (int(x + w), int(y + h))
      color = self._color(track.state)
      cv2.rectangle(self.annotated_frame, p1, p2, color, self.bbox_line_thickness, 1)
      cv2.putText(self.annotated_frame, str(track.id), (p1[0], p1[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, color, self.font_thickness)
      self._add_tracked_path(track, self.annotated_frame)
      self._add_predicted_point(track, self.annotated_frame)

    cv2.putText(self.annotated_frame, self.status_message, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_colour, self.font_thickness)
    cv2.putText(self.annotated_frame, f'(Sky360) Frame count: {self.annotated_frame_count}, Tracked frame count: {data.frame_count}, In sync: {self.annotated_frame_count == data.frame_count}', (25, 50), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_colour, self.font_thickness)
    cv2.putText(self.annotated_frame, self.frame_message, (25, 75), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_colour, self.font_thickness)

    frame_annotated_msg = Frame()
    frame_annotated_msg.epoch = data.epoch
    frame_annotated_msg.fps = data.fps
    frame_annotated_msg.frame_count = data.frame_count
    frame_annotated_msg.frame = self.br.cv2_to_imgmsg(self.annotated_frame)

    self.pub_annotated_frame.publish(frame_annotated_msg)

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

        self.font_size = self.app_configuration['visualiser_font_size']
        self.font_thickness = self.app_configuration['visualiser_font_thickness']
        self.bbox_line_thickness = self.app_configuration['visualiser_bbox_line_thickness']

        self.frame_type = self.app_configuration['visualiser_frame_source']

        self.configuration_loaded = True

  def _load_config(self):
    #self.get_logger().info(f'Loading configuration list.')

    response = self.configuration_svc.send_request(self.configuration_list)
    for config_item in response.entries:
      self.app_configuration[config_item.key] = ConfigEntryConvertor.Convert(config_item.type, config_item.value)

  def _validate_config(self):
    #self.get_logger().info(f'Validating configuration.')

    valid = True

    if self.app_configuration['visualiser_font_size'] == None:
      self.get_logger().error('The visualiser_font_size config entry is null')
      valid = False

    if self.app_configuration['visualiser_font_thickness'] == None:
      self.get_logger().error('The visualiser_font_thickness config entry is null')
      valid = False

    return valid    

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