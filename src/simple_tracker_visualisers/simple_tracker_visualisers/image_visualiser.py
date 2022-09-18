import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from simple_tracker_interfaces.msg import Frame
from simple_tracker_interfaces.msg import TrackingState
from simple_tracker_interfaces.msg import TrackArray
from simple_tracker_interfaces.msg import Track
from simple_tracker_interfaces.msg import BoundingBox
from simple_tracker_interfaces.msg import ConfigEntryUpdatedArray
from .config_entry_convertor import ConfigEntryConvertor
from .configurations_client_async import ConfigurationsClientAsync
from cv_bridge import CvBridge
import cv2
 
class ImageVisualiserNode(Node):

  PROVISIONARY_TARGET = 1
  ACTIVE_TARGET = 2
  LOST_TARGET = 3

  def __init__(self):
    super().__init__('image_visualiser_node')

    self.configuration_list = ['visualiser_font_size', 'visualiser_font_thickness', 'visualiser_bbox_line_thickness']
    self.app_configuration = {}
    self.configuration_loaded = False

    self.configuration_svc = ConfigurationsClientAsync()
    self.sub_config_updated = self.create_subscription(ConfigEntryUpdatedArray, 'sky360/config/updated/v1', self.config_updated_callback, 10)
    #self.camera_original_sub = self.create_subscription(Image, 'sky360/camera/original/v1', self.camera_original_callback, 10)
    self.fp_original_sub = self.create_subscription(Frame, 'sky360/frames/original/v1', self.fp_original_callback, 10)
    #self.fp_grey_sub = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.fp_grey_callback, 10)
    #self.dof_sub = self.create_subscription(Frame, 'sky360/frames/dense_optical_flow/v1', self.dof_callback, 10)
    #self.forground_sub = self.create_subscription(Frame, 'sky360/frames/foreground_mask/v1', self.foreground_callback, 10)
    #self.masked_background_sub = self.create_subscription(Frame, 'sky360/frames/masked_background/v1', self.masked_background_callback, 10)
    self.tracking_state_sub = self.create_subscription(TrackingState, 'sky360/tracker/tracking_state/v1', self.tracking_state_callback, 10)
    self.tracker_tracks_sub = self.create_subscription(TrackArray, 'sky360/tracker/tracks/v1', self.tracks_callback, 10)

    self.br = CvBridge()
    self.annotated_frame = None
    self.annotated_frame_count = 0

    #self.frame_buffer = {}
    self.font_colour = (50, 170, 50)
    self.font_size = 1
    self.font_thickness = 1
    self.bbox_line_thickness = 1
    self.status_message = ''

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_original_callback(self, data:Image):
    #self.get_logger().info('Receiving video frame')
    camera_original_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("camera/original", camera_original_frame)
    cv2.waitKey(1)

  def fp_original_callback(self, data:Frame):

    # TODO: This configuration update thing needs to happen in the background
    if not self.configuration_loaded:
      self._load_and_validate_config()
      self.configuration_loaded = True

    self.annotated_frame = self.br.imgmsg_to_cv2(data.frame)
    self.annotated_frame_count = data.frame_count
    #original_frame = self.br.imgmsg_to_cv2(data.frame)
    #cv2.imshow("fp/original", original_frame)
    #cv2.waitKey(1)

  def fp_grey_callback(self, data:Frame):
    #self.get_logger().info('Receiving video frame')
    camera_grey_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("fp/grey", camera_grey_frame)
    cv2.waitKey(1)

  def dof_callback(self, data:Frame):
    #self.get_logger().info('Receiving video frame')
    dof_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("dense-optical-flow", dof_frame)
    cv2.waitKey(1)

  def foreground_callback(self, data:Frame):
    #self.get_logger().info('Receiving video frame')
    foreground_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("foreground-mask", foreground_frame)
    cv2.waitKey(1)

  def masked_background_callback(self, data:Frame):
    #self.get_logger().info('Receiving video frame')
    masked_background_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("masked-background", masked_background_frame)
    cv2.waitKey(1)

  def tracking_state_callback(self, data:TrackingState):
    msg = f"(Sky360) Tracker Status: trackable:{data.trackable}, alive:{data.alive}, started:{data.started}, ended:{data.ended}, frame count:{data.frame_count}"
    self.status_message = msg
    #self.get_logger().info(msg)

  def tracks_callback(self, data:TrackArray):

    if self.configuration_loaded == False:
      pass

    for track in data.tracks:
      bbox: BoundingBox = track.bbox
      x = bbox.x
      y = bbox.y
      w = bbox.w
      h = bbox.h
      p1 = (int(x), int(y))
      p2 = (int(x + w), int(y + h))
      color = self._color(track.state)
      cv2.rectangle(self.annotated_frame, p1, p2, color, self.bbox_line_thickness, 1)
      cv2.putText(self.annotated_frame, str(track.id), (p1[0], p1[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, color, self.font_thickness)

    cv2.putText(self.annotated_frame, self.status_message, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_colour, self.font_thickness)
    cv2.putText(self.annotated_frame, f'(Sky360) Frame count: {self.annotated_frame_count}, Tracked frame count: {data.frame_count}, In sync: {self.annotated_frame_count == data.frame_count}', (25, 50), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_colour, self.font_thickness)
    cv2.putText(self.annotated_frame, '(Sky360) Original Frame', (25, 75), cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_colour, self.font_thickness)

    cv2.imshow("annotated", self.annotated_frame)
    cv2.waitKey(1)

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

  def _color(self, tracking_state):
    return {
            self.PROVISIONARY_TARGET: (25, 175, 175),
            self.ACTIVE_TARGET: (50, 170, 50),
            self.LOST_TARGET: (50, 50, 225)
        }[tracking_state]


def main(args=None):

  rclpy.init(args=args)
  image_visualiser = ImageVisualiserNode()
  rclpy.spin(image_visualiser)
  image_visualiser.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()