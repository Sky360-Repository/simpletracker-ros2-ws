import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
class ImageVisualiserNode(Node):

  def __init__(self):
    super().__init__('image_visualiser')
    self.subscription = self.create_subscription(Image, 'sky360/camera/frame/original/v1', self.listener_callback, 10)
    self.subscription
    self.br = CvBridge()
   
  def listener_callback(self, data):
    #self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("camera/original", current_frame)
    cv2.waitKey(1)

def main(args=None):

  rclpy.init(args=args)
  image_visualiser = ImageVisualiserNode()
  rclpy.spin(image_visualiser)
  image_visualiser.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()