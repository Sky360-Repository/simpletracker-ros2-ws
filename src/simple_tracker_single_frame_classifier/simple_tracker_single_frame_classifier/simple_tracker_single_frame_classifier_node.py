# Subscribe to topic: ros2 topic echo /sky360/classification
# TensorFlow version 2.12.0 worked

import traceback as tb
import rclpy
import message_filters
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles
from typing import List
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import ObjectHypothesis, Detection2DArray, Classification
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import TrackingState, TrackTrajectoryArray
# from simple_tracker_interfaces.msg import ClassifiedObject, ClassifiedObjectsArray
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
from simple_tracker_annotated_frame_provider.annotated_frame_creator import AnnotatedFrameCreator
from tensorflow.keras.preprocessing import image as kimage
from tensorflow.keras.models import load_model
from PIL import Image as ImageROI
import cv2

class SingleFrameClassifierNode(Node):
    def __init__(self, model_path):
        super().__init__('single_frame_classifier')
        # assuming 'settings' is a dictionary with the necessary settings
        settings = {
                'visualiser_bbox_line_thickness': 2,
                'visualiser_frame_source': 'source',
                'visualiser_show_cropped_tracks': True,
                'visualiser_cropped_zoom_factor': 1.5,
                'visualiser_bbox_size': None
        }
        self.creator = AnnotatedFrameCreator(settings)
        self.bridge = CvBridge()

        # Load the Keras model
        self.model = load_model(model_path)

        subscriber_qos_profile = get_topic_subscriber_qos_profile()
        publisher_qos_profile = get_topic_publisher_qos_profile()

        # Add a publisher for the classification results
        self.pub_classification = self.create_publisher(String, 'sky360/classification', publisher_qos_profile)

        self.sub_masked_frame = message_filters.Subscriber(self, Image, 'sky360/frames/masked', qos_profile=subscriber_qos_profile)
        self.sub_tracking_state = message_filters.Subscriber(self, TrackingState, 'sky360/tracker/tracking_state', qos_profile=subscriber_qos_profile)
        self.sub_tracker_detections = message_filters.Subscriber(self, Detection2DArray, 'sky360/tracker/detections', qos_profile=subscriber_qos_profile)
        self.sub_tracker_trajectory = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/trajectory', qos_profile=subscriber_qos_profile)
        self.sub_tracker_prediction = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/prediction', qos_profile=subscriber_qos_profile)

        # TimeSynchronizer to ensure all messages have the same timestamp
        self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_tracking_state,
        self.sub_tracker_detections, self.sub_tracker_trajectory, self.sub_tracker_prediction], 10)
        self.time_synchronizer.registerCallback(self.synced_callback)

    def synced_callback(self, msg_masked_frame, msg_tracking_state, msg_detection_array, msg_trajectory_array, msg_prediction_array):
        # Convert the ROS Image message to a CV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg_masked_frame, "bgr8")

        for detection in msg_detection_array.detections:
            # Extract the region defined by the bounding box
            bbox = self.creator._get_sized_bbox(detection.bbox)
            # Convert the bounding box coordinates to integers
            x, y, w, h = map(int, bbox)
            img = cv2.cvtColor(cv_image[y:y+h, x:x+w], cv2.COLOR_BGR2RGB)
            img = ImageROI.fromarray(img)
            img = img.resize((64, 64))
            img_array = kimage.img_to_array(img)
            img_array = np.expand_dims(img_array, axis=0)
            # Classify the region and publish the result
            predictions = self.model.predict(img_array)
            # Log the classification
            self.get_logger().info('Model predictions: ' + str(predictions))
            # TODO: Convert predictions to a suitable format for your message type
            # Here we convert the numpy array to a string
            msg_classification = String(data=str(predictions))
            self.pub_classification.publish(msg_classification)

def main(args=None):
    rclpy.init(args=args)
    # Get current working directory
    workspace_root = os.getcwd()
    # Construct the path to your model
    model_path = os.path.join(workspace_root, 'src', 'simple_tracker_single_frame_classifier', 'models', 'cnn-mlp-05.keras')
    node = SingleFrameClassifierNode(model_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import traceback as tb
# import rclpy
# import message_filters
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSPresetProfiles
# from typing import List
# from std_msgs.msg import String, Float64MultiArray
# from geometry_msgs.msg import Point
# from sensor_msgs.msg import Image, CompressedImage
# from vision_msgs.msg import ObjectHypothesis, Detection2DArray, Classification, ObjectHypothesisWithPose
# from cv_bridge import CvBridge
# from simple_tracker_interfaces.msg import TrackingState, TrackTrajectoryArray, ClassificationWithBBox, ClassificationWithBBoxArray
# from simple_tracker_shared.configured_node import ConfiguredNode
# from simple_tracker_shared.node_runner import NodeRunner
# from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile

# import os
# from ament_index_python.packages import get_package_share_directory
# import numpy as np
# from simple_tracker_annotated_frame_provider.annotated_frame_creator import AnnotatedFrameCreator
# from tensorflow.keras.preprocessing import image as kimage
# from tensorflow.keras.models import load_model
# from PIL import Image as ImageROI
# import cv2


# class SingleFrameClassifierNode(Node):
#     def __init__(self, model_path):
#         super().__init__('single_frame_classifier')
#         settings = {
#                 'visualiser_bbox_line_thickness': 2,
#                 'visualiser_frame_source': 'source',
#                 'visualiser_show_cropped_tracks': True,
#                 'visualiser_cropped_zoom_factor': 1.5,
#                 'visualiser_bbox_size': None
#         }
#         self.creator = AnnotatedFrameCreator(settings)
#         self.bridge = CvBridge()

#         self.model = load_model(model_path)

#         subscriber_qos_profile = get_topic_subscriber_qos_profile()
#         publisher_qos_profile = get_topic_publisher_qos_profile()

#         self.pub_classification = self.create_publisher(ClassificationWithBBoxArray, 'sky360/classifications', publisher_qos_profile)

#         self.sub_masked_frame = message_filters.Subscriber(self, Image, 'sky360/frames/masked', qos_profile=subscriber_qos_profile)
#         self.sub_tracking_state = message_filters.Subscriber(self, TrackingState, 'sky360/tracker/tracking_state', qos_profile=subscriber_qos_profile)
#         self.sub_tracker_detections = message_filters.Subscriber(self, Detection2DArray, 'sky360/tracker/detections', qos_profile=subscriber_qos_profile)
#         self.sub_tracker_trajectory = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/trajectory', qos_profile=subscriber_qos_profile)
#         self.sub_tracker_prediction = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/prediction', qos_profile=subscriber_qos_profile)

#         self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_tracking_state,
#         self.sub_tracker_detections, self.sub_tracker_trajectory, self.sub_tracker_prediction], 10)
#         self.time_synchronizer.registerCallback(self.synced_callback)

#     def synced_callback(self, msg_masked_frame, msg_tracking_state, msg_detection_array, msg_trajectory_array, msg_prediction_array):
#         cv_image = self.bridge.imgmsg_to_cv2(msg_masked_frame, "bgr8")

#         msg_classifications = ClassificationWithBBoxArray()

#         for detection in msg_detection_array.detections:
#             # Extract the region defined by the bounding box
#             bbox = self.creator._get_sized_bbox(detection.bbox)
#             # Convert the bounding box coordinates to integers
#             x, y, w, h = map(int, bbox)
#             img = cv2.cvtColor(cv_image[y:y+h, x:x+w], cv2.COLOR_BGR2RGB)
#             img = ImageROI.fromarray(img)
#             img = img.resize((64, 64))
#             img_array = kimage.img_to_array(img)
#             img_array = np.expand_dims(img_array, axis=0)
#             # Classify the region and publish the result
#             predictions = self.model.predict(img_array)
#             # Log the classification
#             # self.get_logger().info('Model predictions: ' + str(predictions))

#             msg_classification = ClassificationWithBBox()
#             msg_classification.classification.data = predictions.flatten().tolist()
#             # Here we convert the numpy array to a string
#         #     msg_classification = String(data=str(predictions))
#         #     msg_classification.center = Point(detection.bbox.center.position.x, detection.bbox.center.position.y, 0)

#             msg_classification.center.x = detection.bbox.center.position.x
#             msg_classification.center.y = detection.bbox.center.position.y
#             msg_classification.center.z = 0

#             msg_classification.size = detection.bbox.size

#         #     self.pub_classification.publish(msg_classification)

#             msg_classifications.classifications.append(msg_classification)

#         self.pub_classification.publish(msg_classifications)

# def main(args=None):
#     rclpy.init(args=args)
#     workspace_root = os.getcwd()
#     model_path = os.path.join(workspace_root, 'src', 'simple_tracker_single_frame_classifier', 'models', 'cnn-mlp-05.keras')
#     node = SingleFrameClassifierNode(model_path)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
