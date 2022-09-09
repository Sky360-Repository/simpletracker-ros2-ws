import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .config_entry_convertor import ConfigEntryConvertor
from .configuration_client_async import ConfigurationClientAsync

class ResizedImagePublisherNode(Node):

  def __init__(self, configuration_client):

    super().__init__('resized_image_publisher')  

    self.config = configuration_client

    # string
    response = configuration_client.send_request('camera_uri')
    self.get_logger().info(f'camera_uri --> type [{response.type}] value [{response.value}].')
    self.camera_uri = ConfigEntryConvertor.Convert(response.type, response.value)
    self.get_logger().info(f'Retrieved camera_uri from config {self.camera_uri}.')

    # int
    response = configuration_client.send_request('resize_dimension')
    self.get_logger().info(f'resize_dimension --> type [{response.type}] value [{response.value}].')
    self.resize_dimension = ConfigEntryConvertor.Convert(response.type, response.value)
    self.get_logger().info(f'Retrieved resize_dimension from config {self.resize_dimension}.')

    # bool
    response = configuration_client.send_request('resize_frame')
    self.get_logger().info(f'resize_frame --> type [{response.type}] value [{response.value}].')
    self.resize_frame = ConfigEntryConvertor.Convert(response.type, response.value)
    self.get_logger().info(f'Retrieved resize_frame from config {self.resize_frame}.')

    self.publisher_ = self.create_publisher(Image, 'video_frames_resize', 10)
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.cap = cv2.VideoCapture(self.camera_uri)
    self.br = CvBridge()
    self.get_logger().info(f'ImagePublisherNode up and running.')
   
  def timer_callback(self):
    
    ret, frame = self.cap.read()
          
    if ret == True:
      frame = cv2.resize(frame, (960, 960))
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
 
    # Display the message on the console
    self.get_logger().info('Publishing resized video frame')
  
def main(args=None):

  rclpy.init(args=args)
  configuration_client = ConfigurationClientAsync()
  resized_image_publisher = ResizedImagePublisherNode(configuration_client)
  rclpy.spin(resized_image_publisher)
  resized_image_publisher.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()