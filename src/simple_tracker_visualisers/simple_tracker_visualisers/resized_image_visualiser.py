import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
 
class ResizedImageVisualiserNode(Node):

  def __init__(self):
    super().__init__('resized_image_subscriber')
    self.subscription = self.create_subscription(Image, 'sky360/frame/original/scaled/v1', self.listener_callback, 10)
    self.subscription
    self.br = CvBridge()
   
  def listener_callback(self, data):
    #self.get_logger().info('Receiving resized video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("camera-resize", current_frame)
    cv2.waitKey(1)

def main(args=None):

  rclpy.init(args=args)
  resized_image_visualiser = ResizedImageVisualiserNode()
  rclpy.spin(resized_image_visualiser)
  resized_image_visualiser.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()