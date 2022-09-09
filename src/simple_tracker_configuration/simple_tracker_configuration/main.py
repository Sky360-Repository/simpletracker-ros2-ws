import rclpy
from .configuration_node import SimpleTrackerConfigurationNode

def main(args=None):
  rclpy.init(args=args)
  configuration_service = SimpleTrackerConfigurationNode()  
  rclpy.spin(configuration_service)
  configuration_service.destroy_node()
  rclpy.rosshutdown()

if __name__ == '__main__':
  main()