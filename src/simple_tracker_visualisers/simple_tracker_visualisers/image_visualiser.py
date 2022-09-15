import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
class ImageVisualiserNode(Node):

  def __init__(self):
    super().__init__('image_visualiser')
    #self.camera_original_sub = self.create_subscription(Image, 'sky360/camera/original/v1', self.camera_original_callback, 10)
    self.fp_original_sub = self.create_subscription(Image, 'sky360/frames/original/v1', self.fp_original_callback, 10)
    #self.fp_grey_sub = self.create_subscription(Image, 'sky360/frames/grey/v1', self.fp_grey_callback, 10)
    #self.dof_sub = self.create_subscription(Image, 'sky360/frames/dense_optical_flow/v1', self.dof_callback, 10)
    #self.forground_sub = self.create_subscription(Image, 'sky360/frames/foreground_mask/v1', self.forground_callback, 10)
    #self.masked_background_sub = self.create_subscription(Image, 'sky360/frames/masked_background/v1', self.masked_background_callback, 10)

    self.br = CvBridge()
   
  def camera_original_callback(self, data):
    #self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("camera/original", current_frame)
    cv2.waitKey(1)

  def fp_original_callback(self, data):
    #self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("fp/original", current_frame)
    cv2.waitKey(1)

  def fp_grey_callback(self, data):
    #self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("fp/grey", current_frame)
    cv2.waitKey(1)

  def dof_callback(self, data):
    #self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("dense-optical-flow", current_frame)
    cv2.waitKey(1)         

  def forground_callback(self, data):
    #self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("foreground-mask", current_frame)
    cv2.waitKey(1)  

  def masked_background_callback(self, data):
    #self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    cv2.imshow("masked-background", current_frame)
    cv2.waitKey(1)  

def main(args=None):

  rclpy.init(args=args)
  image_visualiser = ImageVisualiserNode()
  rclpy.spin(image_visualiser)
  image_visualiser.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()