import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .setting_convertor import SettingConvertor
from .configuration_client_async import ConfigurationClientAsync

class ImagePublisherNode(Node):

  def __init__(self, configuration_client):

    super().__init__('image_publisher')  

    self.config = configuration_client

    # string
    response = configuration_client.send_request('camera_uri')
    self.get_logger().info(f'camera_uri --> type [{response.type}] value [{response.value}].')
    camera_uri = SettingConvertor.Convert(response.type, response.value)
    self.get_logger().info(f'Retrieved camera_uri from config {camera_uri}.')

    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.cap = cv2.VideoCapture(camera_uri)
    self.br = CvBridge()
    self.get_logger().info(f'ImagePublisherNode up and running.')
   
  def timer_callback(self):

    ret, frame = self.cap.read()
          
    if ret == True:
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
 
    # Display the message on the console
    self.get_logger().info('Publishing video frame')
  
def main(args=None):

  rclpy.init(args=args)
  configuration_client = ConfigurationClientAsync()
  image_publisher = ImagePublisherNode(configuration_client)
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()