import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .config_entry_convertor import ConfigEntryConvertor
from .configuration_client_async import ConfigurationClientAsync
from .configurations_client_async import ConfigurationsClientAsync

class ImagePublisherNode(Node):

  def __init__(self, configuration_client, configurations_client):

    super().__init__('image_publisher')  

    self.config_svc = configuration_client
    self.configs_svc = configurations_client

    # config string example
    self.get_logger().info(f'Testing single config request.')

    response = configuration_client.send_request('camera_uri')
    self.get_logger().info(f'camera_uri --> type [{response.entry.type}] value [{response.entry.value}].')
    self.camera_uri = ConfigEntryConvertor.Convert(response.entry.type, response.entry.value)
    self.get_logger().info(f'Retrieved camera_uri from config {self.camera_uri}.')

    # config int example
    response = configuration_client.send_request('resize_dimension_h')
    self.get_logger().info(f'resize_dimension --> type [{response.entry.type}] value [{response.entry.value}].')
    self.resize_dimension_h = ConfigEntryConvertor.Convert(response.entry.type, response.entry.value)
    self.get_logger().info(f'Retrieved resize_dimension from config {self.resize_dimension_h}.')

    # config bool example
    response = configuration_client.send_request('resize_frame')
    self.get_logger().info(f'resize_frame --> type [{response.entry.type}] value [{response.entry.value}].')
    self.resize_frame = ConfigEntryConvertor.Convert(response.entry.type, response.entry.value)
    self.get_logger().info(f'Retrieved resize_frame from config {self.resize_frame}.')

    
    
    # configs example
    self.get_logger().info(f'Testing a config list request.')

    configs = ['camera_uri', 'resize_dimension_h', 'resize_frame']
    response = configurations_client.send_request(configs)
    for config_item in response.entries:
      self.get_logger().info(f'{config_item.key} --> type [{config_item.type}] value [{config_item.value}].')

    # setup publishers
    self.pub_full_frame = self.create_publisher(Image, 'sky360/frames/original', 10)
    self.pub_resized_frame = self.create_publisher(Image, 'sky360/frames/original/resized', 10)

    # setup timer and other helpers
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.capture = cv2.VideoCapture(self.camera_uri)
    self.br = CvBridge()
    self.get_logger().info(f'ImagePublisherNode up and running.')
   
  def timer_callback(self):

    success, frame = self.capture.read()          
    if success == True:      

      self.pub_full_frame.publish(self.br.cv2_to_imgmsg(frame))
      #resized_frame = self.frame_resize(frame, height=self.resize_dimension_h)
      resized_frame = cv2.resize(frame, (self.resize_dimension_h,self.resize_dimension_h), interpolation=cv2.INTER_AREA)
      self.pub_resized_frame.publish(self.br.cv2_to_imgmsg(resized_frame))
 
      # Display the message on the console
      #self.get_logger().info('Publishing video frame')

  def frame_resize(frame, width=None, height=None, inter=cv2.INTER_AREA):
    # initialize the dimensions of the frame to be resized and
    # grab the frame size
    dim = None
    (h, w) = frame.shape[:2]

    # if both the width and height are None, then return the
    # original frame
    if width is None and height is None:
        return frame

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the frame
    resized = cv2.resize(frame, dim, interpolation = inter)

    # return the resized frame
    return resized
  
def main(args=None):

  rclpy.init(args=args)
  configuration_client = ConfigurationClientAsync()
  configurations_client = ConfigurationsClientAsync()
  image_publisher = ImagePublisherNode(configuration_client, configurations_client)
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()