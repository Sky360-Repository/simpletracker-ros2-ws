import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from simple_tracker_interfaces.msg import Frame
from simple_tracker_interfaces.msg import TrackingState
from simple_tracker_interfaces.msg import TrackArray
from simple_tracker_interfaces.msg import Track
from simple_tracker_interfaces.msg import BoundingBox
from cv_bridge import CvBridge
import cv2
 
class ImageVisualiserNode(Node):

  PROVISIONARY_TARGET = 1
  ACTIVE_TARGET = 2
  LOST_TARGET = 3

  def __init__(self):
    super().__init__('image_visualiser_node')
    self.camera_original_sub = self.create_subscription(Image, 'sky360/camera/original/v1', self.camera_original_callback, 10)
    self.fp_original_sub = self.create_subscription(Frame, 'sky360/frames/original/v1', self.fp_original_callback, 10)
    self.fp_grey_sub = self.create_subscription(Frame, 'sky360/frames/grey/v1', self.fp_grey_callback, 10)
    self.dof_sub = self.create_subscription(Frame, 'sky360/frames/dense_optical_flow/v1', self.dof_callback, 10)
    self.forground_sub = self.create_subscription(Frame, 'sky360/frames/foreground_mask/v1', self.foreground_callback, 10)
    self.masked_background_sub = self.create_subscription(Frame, 'sky360/frames/masked_background/v1', self.masked_background_callback, 10)
    self.tracking_state_sub = self.create_subscription(TrackingState, 'sky360/tracker/tracking_state/v1', self.tracking_state_callback, 10)
    self.tracker_tracks_sub = self.create_subscription(TrackArray, 'sky360/tracker/tracks/v1', self.tracks_callback, 10)

    self.br = CvBridge()
    self.annotated_frame = None

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_original_callback(self, data:Image):
    #self.get_logger().info('Receiving video frame')
    camera_original_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("camera/original", camera_original_frame)
    cv2.waitKey(1)

  def fp_original_callback(self, data:Frame):
    #self.get_logger().info('Receiving video frame')
    self.annotated_frame = self.br.imgmsg_to_cv2(data.frame)
    original_frame = self.br.imgmsg_to_cv2(data.frame)
    cv2.imshow("fp/original", original_frame)
    cv2.waitKey(1)

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
    msg = f"Trackers: trackable:{data.trackable}, alive:{data.alive}, started:{data.started}, ended:{data.ended}, frame count:{data.frame_count} (Sky360)"
    self.get_logger().info(msg)

  def tracks_callback(self, data:TrackArray):
    for track in data.tracks:
      bbox: BoundingBox = track.bbox
      x = bbox.x
      y = bbox.y
      w = bbox.w
      h = bbox.h
      p1 = (int(x), int(y))
      p2 = (int(x + w), int(y + h))
      color = self.color(track.state)
      cv2.rectangle(self.annotated_frame, p1, p2, color, 2, 1)
      cv2.putText(self.annotated_frame, str(track.id), (p1[0], p1[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    cv2.imshow("annotated", self.annotated_frame)
    cv2.waitKey(1)

  def color(self, tracking_state):
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